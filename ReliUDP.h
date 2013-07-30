#pragma once
#pragma pack(4)

#include <iostream>
#include <WinSock2.h>
#include <process.h>
#include <list>
#include <vector>
#include <queue>
#include <time.h>
#include <Windows.h>

#define CHECK_SUM

typedef unsigned int uint32_t;
typedef unsigned short uint16_t;

static const int MaxThread = 12;

static const int OSBufferSize = 65536;		//OS bufferSize default 64k

static const int FragmentDataSize = 512 * 8;		//default 512*8
static const int FragmentHeaderSize = 20;
static const int FragmentSize = FragmentHeaderSize + FragmentDataSize;

static const int TimeWait = int(12.0 * double(CLOCKS_PER_SEC) / 1000.0);					//default 8
static const int TimeWaitSizeFactor = int((0.002 * (double(FragmentDataSize) / 10240.0)) * double(CLOCKS_PER_SEC) / 1000.0);	//(0.002*(double(FRAGMENT_DATA_SIZE)/10240.0))			ms/frame

static const int SendSampleSize = 1024 * 24;	//default 24k

static const int SendTimeout = int(30000 * double(CLOCKS_PER_SEC) / 1000.0); //default 30k ms
static const int SendTimeoutFactor = int(50 * double(CLOCKS_PER_SEC) / 1000.0); //default 50  ms/frame
static const int SendNoReceiverTimeout = int(10000 * double(CLOCKS_PER_SEC) / 1000.0);	//default 1 s

static const double ExpectRate = .99; 	//default 99%
static const int ExpectTimeout = int(32 * double(CLOCKS_PER_SEC) / 1000.0);		//default 32 ms
static const int ExpectExceptionSize = 12;		//default 12

static const int RecvSeqIDBufferSize = 320; //default 200

static const int RecvBUFWait = int(5 * double(CLOCKS_PER_SEC) / 1000.0); //default 5ms

static const int FragmentTimeout = int(30000 * double(CLOCKS_PER_SEC) / 1000.0); //default 30k ms
static const int FragmentTimeoutFactor = int(2 * double(CLOCKS_PER_SEC) / 1000.0);		//default 2ms/frame

#define FRAGMENT_DATA 7953
#define FRAGMENT_RESPONSE 8761
#define FRAGMENT_INVALID 5479
#define FRAGMENT_RESET 4913
#define FRAGMENT_RESET_RESPONSE 9481


#define ID_ERROR -2

#define SOCK_INIT_FAIL false
#define SOCK_INIT_OK true

#define SEND_BLOCK 1
#define SEND_UNBLOCK 0
#define SEND_BLOCK_CHECK 1	//bit 0

using namespace std;


typedef struct _fragmentST {
    uint16_t type;			//DATA / RESPONSE / INVALID
    uint16_t messageSeqID;
    uint16_t fragmentID;
    uint16_t fragmentNum;
    uint32_t IPort;			//IP & Port  :x.xx.xxx.xxxx:pp =>xxx.xxxx:pp [16+16]
    uint32_t dataSize;
    uint32_t checkSum;
    char data[FragmentDataSize];
} fragment;

static void addrCopy(SOCKADDR_IN *dst, const SOCKADDR_IN src) {
    memset(dst, 0, sizeof(SOCKADDR_IN));
    dst->sin_family = src.sin_family;
    dst->sin_addr.S_un.S_addr = src.sin_addr.S_un.S_addr;
    dst->sin_port = src.sin_port;
}

class bitSet {
public:
    bitSet(int n) {
        N = n;
        size = (N >> 3) + ((N & 7) != 0); //about N/8  bytes
        dat = new char[size];
        memset(dat, 0, size);
    }
    ~bitSet() {
        delete[] dat;
    }
    void set(int pos) {
        if(pos < N)
            dat[pos >> 3] |= 1 << (pos & 7);
    }
    void reset(int pos) {
        if(pos < N)
            dat[pos >> 3] &= -1 ^ (1 << (pos & 7));
    }
    void allSet() {
        memset(dat, -1, size);
    }
    void allReset() {
        memset(dat, 0, size);
    }
    bool check(int pos) {
        return pos < N ? (dat[pos >> 3] & (1 << (pos & 7))) != 0 : false;
    }
    bool bigCheck(int posB) {	//check 1 byte as 8-bit ...faster than check 8 single bit
        return posB < size ? (dat[posB]) != 0 : false;
    }
    int getOne() {
        int i;
        for(i = 0; i < (N >> 3); ++i)
            if(bigCheck(i))
                for(int j = i << 3; j < (i << 3) + 8; ++j)
                    if(check(j))
                        return j;
        for(i <<= 3; i < N; ++i)
            if(check(i))
                return i;
        return -1;
    }
    vector<int> getAll() {
        vector<int> res;
        int i;
        for(i = 0; i < (N >> 3); ++i)
            if(bigCheck(i))
                for(int j = i << 3; j < (i << 3) + 8; ++j)
                    if(check(j))
                        res.push_back(j);
        for(i <<= 3; i < N; ++i)
            if(check(i))
                res.push_back(i);
        return res;
    }
private:
    char *dat;
    int N;
    int size;
};


class statEntry {
public:
    statEntry(uint32_t iport, int i, int n) {
        IPort = iport;
        id = i;
        bits = new bitSet(n);
    }
    ~statEntry() {
        delete bits;
    }
    uint32_t IPort;
    int id;
    bitSet *bits;
};

class IPortSeq {
public:
    IPortSeq(uint32_t iport, uint32_t seq) {
        IPort = iport;
        SeqID = seq;
    }
    uint32_t IPort;
    uint16_t SeqID;
};

class sendStat {
public:
    sendStat() {
        indexSet.clear();
        seqIndex.clear();
    }
    ~sendStat() {
        indexSet.clear();
        seqIndex.clear();
    }
    list<statEntry>::iterator find(fragment *frame) {
        for(list<statEntry>::iterator it = indexSet.begin(); it != indexSet.end(); ++it) {
            if(it->IPort == frame->IPort && it->id == frame->messageSeqID)
                return it;
        }
        return indexSet.end();
    }
    void newSeq(fragment *frame) {
        list<statEntry>::iterator it = find(frame);
        if(it == indexSet.end()) {
            statEntry *entry = new statEntry(frame->IPort, frame->messageSeqID, frame->fragmentNum);
            indexSet.push_back(*entry);
        }
    }
    void set(fragment *frame) {
        list<statEntry>::iterator it = find(frame);
        if(it != indexSet.end())
            it->bits->set(frame->fragmentID);
    }
    void reset(fragment *frame) {
        list<statEntry>::iterator it = find(frame);
        if(it != indexSet.end())
            it->bits->reset(frame->fragmentID);
    }
    void allSet(fragment *frame) {
        list<statEntry>::iterator it = find(frame);
        if(it != indexSet.end())
            it->bits->allSet();
    }
    void allReset(fragment *frame) {
        list<statEntry>::iterator it = find(frame);
        if(it != indexSet.end())
            it->bits->allReset();
    }
    int getOne(fragment *frame) {
        list<statEntry>::iterator it = find(frame);
        if(it != indexSet.end())
            return it->bits->getOne();
        return ID_ERROR;
    }
    vector<int> getAll(fragment *frame) {
        list<statEntry>::iterator it = find(frame);
        if(it != indexSet.end())
            return it->bits->getAll();
        vector<int> empty;
        return empty;
    }
    void removeSeq(fragment *frame) {
        list<statEntry>::iterator it = find(frame);
        if(it != indexSet.end()) {
            indexSet.erase(it);
        }
    }

    list<IPortSeq>::iterator findSeq(uint32_t iport) {
        for(list<IPortSeq>::iterator it = seqIndex.begin(); it != seqIndex.end(); ++it) {
            if(it->IPort == iport)
                return it;
        }
        return seqIndex.end();
    }
    bool haveIPort(uint32_t iport) {
        return findSeq(iport) != seqIndex.end();
    }
    void newIPortSeq(uint32_t iport) {
        IPortSeq *entry = new IPortSeq(iport, 1);
        seqIndex.push_back(*entry);
    }

    uint32_t getIPortSeq(uint32_t iport) {
        list<IPortSeq>::iterator it = findSeq(iport);
        if(it != seqIndex.end()) {
            return ++it->SeqID;
        }
        else {
            newIPortSeq(iport);
            return 1;
        }
    }

private:
    list<statEntry> indexSet;
    list<IPortSeq> seqIndex;
};



class recvEntry {
public:
    recvEntry(uint32_t iport, int i, int n, int dataSize) {
        IPort = iport;
        id = i;
        size = n;
        bits = new bitSet(n);
        bits->allSet();	//initial 0xFF
        data = new char[dataSize];
        timeoutTime = clock() + FragmentTimeout + FragmentTimeoutFactor * n;
    }
    ~recvEntry() {
        delete bits;
        delete[] data;
    }
    bool checkTimeout(clock_t now) {
        return now > timeoutTime || now + 1200000 < timeoutTime;	//obvious timoutOut or overflow => timeout
    }
public:
    uint32_t IPort;
    uint16_t id;
    bitSet *bits;
    char *data;
    clock_t timeoutTime;
    int size;
};


class recvStat {
public:
    recvStat() {
        indexSet.clear();
        receivedSeqSet.clear();
    }
    ~recvStat() {
        indexSet.clear();
        receivedSeqSet.clear();
    }
    void addSeqId(fragment *frame) {
        IPortSeq *entry = new IPortSeq(frame->IPort, frame->messageSeqID);
        receivedSeqSet.push_back(*entry);
        if(receivedSeqSet.size() > RecvSeqIDBufferSize)	//buffer overflow remove head
            receivedSeqSet.erase(receivedSeqSet.begin());
    }
    bool checkSeqId(fragment *frame) {
        for(list<IPortSeq>::iterator it = receivedSeqSet.begin(); it != receivedSeqSet.end(); ++it)
            if(it->IPort == frame->IPort && it->SeqID == frame->messageSeqID)
                return true;
        return false;
    }
    list<recvEntry>::iterator find(fragment *frame) {
        for(list<recvEntry>::iterator it = indexSet.begin(); it != indexSet.end(); ++it) {
            if(it->IPort == frame->IPort && it->id == frame->messageSeqID)
                return it;
        }
        return indexSet.end();
    }
    bool have(fragment *frame) {
        return find(frame) != indexSet.end();
    }

    void newSeq(fragment *frame) {
        list<recvEntry>::iterator it = find(frame);
        if(it == indexSet.end()) {
            recvEntry *entry = new recvEntry(frame->IPort, frame->messageSeqID, frame->fragmentNum, frame->dataSize);
            indexSet.push_back(*entry);
        }
    }
    bool check(fragment *frame) {
        list<recvEntry>::iterator it = find(frame);
        if(it != indexSet.end()) {
            return it->bits->check(frame->fragmentID);
        }
        return false;
    }
    void storeData(fragment *frame, int dataLen) {
        list<recvEntry>::iterator it = find(frame);
        if(it != indexSet.end()) {
            memcpy((it->data) + (frame->fragmentID * FragmentDataSize), frame->data, dataLen);
            it->bits->reset(frame->fragmentID);	//reset mark as received
        }
    }
    const char* getDataPointer(fragment *frame) {
        list<recvEntry>::iterator it = find(frame);
        if(it != indexSet.end()) {
            return it->data;
        }
        return NULL;
    }
    int getOne(fragment *frame) {
        list<recvEntry>::iterator it = find(frame);
        if(it != indexSet.end())
            return it->bits->getOne();
        return ID_ERROR;
    }
    void removeSeq(fragment *frame) {
        list<recvEntry>::iterator it = find(frame);
        if(it != indexSet.end()) {
            indexSet.erase(it);
        }
    }
    void clearSeqSet(fragment *frame) {
        list<IPortSeq>::iterator it = receivedSeqSet.begin();
        for(size_t i = 0; i < receivedSeqSet.size(); ++i) {
            if(it->IPort == frame->IPort) {
                receivedSeqSet.erase(it);
                it = receivedSeqSet.begin();
                int t = i;
                while(t--)	++it;
                --i;
            }
            else
                ++it;
        }
    }
    void removeTimeout() {
        clock_t now = clock();
        list<recvEntry>::iterator it = indexSet.begin();
        for(size_t i = 0; i < indexSet.size(); ++i) {
            if(it->checkTimeout(now)) {
#ifdef DEBUG
                cout << "[timeout:" << it->id << "]" << endl;
#endif
                indexSet.erase(it);
                it = indexSet.begin();
                int t = i;
                while(t--)	++it;
                --i;
            }
            else
                ++it;
        }
    }


private:
    list<recvEntry> indexSet;
    list<IPortSeq> receivedSeqSet;
};

class bufEntry {
public:
    bufEntry(int n) {
        data = new char[n];
        dataLength = n;
    }
    ~bufEntry() {
        delete[] data;
    }
    void storeData(const char *src, int dataLen, SOCKADDR_IN add) {
        memcpy(data, src, dataLen);
        addrCopy(&addr, add);
    }
    int popData(char *des, SOCKADDR_IN *addr) {
        memcpy(des, data, dataLength);
        addrCopy(addr, this->addr);
        return dataLength;
    }
    uint32_t getLen() {
        return dataLength;
    }
private:
    char *data;
    uint32_t dataLength;
    SOCKADDR_IN addr;
};

class recvBuffer {
public:
    recvBuffer() {
        clear();
    }
    ~recvBuffer() {
        clear();
    }
    bool empty() {
        return indexSet.empty();
    }
    void addEntry(const char *src, int datalen, SOCKADDR_IN add) {
        bufEntry *entry = new bufEntry(datalen);
        entry->storeData(src, datalen, add);
        indexSet.push(*entry);
    }
    int popEntry(char *des, SOCKADDR_IN *addr) {
        int len = indexSet.front().popData(des, addr);
        indexSet.pop();
        return len;
    }
    void clear() {
        queue<bufEntry> empty;
        swap(empty, indexSet);
    }
    uint32_t getHeadSize() {
        return indexSet.empty() ? 0 : indexSet.front().getLen();
    }

private:
    queue<bufEntry> indexSet;
};

class CRC32 {
public:
    CRC32() {
        const uint32_t Polynomial = 0xEDB88320;
        for (unsigned int i = 0; i <= 0xFF; ++i) {
            uint32_t crc = i;
            for (unsigned int j = 0; j < 8; ++j)
                crc = (crc >> 1) ^ (-int(crc & 1) & Polynomial);
            crc32Lookup[i] = crc;
        }
    }
    uint32_t check(const void* data, size_t length, uint32_t previousCrc32 = 0) {
        uint32_t crc = ~previousCrc32;
        unsigned char* current = (unsigned char*) data;
        while(length--)
            crc = (crc >> 8) ^ crc32Lookup[(crc & 0xFF) ^ *current++];
        return ~crc;
    }
private:
    uint32_t crc32Lookup[256];
};





class ReliUDP {
public:
    ReliUDP(void);
    ~ReliUDP(void);
    bool winSockInit();
    void setLocalAddr(string ip, int port);
    void startCom();
    void stopCom();
    void clearCom();
    void udpSendData(const char *dat, int dataLength, SOCKADDR_IN addr);
    int udpRecvData(char *buf, int dataLength, SOCKADDR_IN *addr);
    void sendData(const char *dat, int dataLength, SOCKADDR_IN addr, char sendOpt = SEND_BLOCK);
    int recvData(char *buf, int dataLength, SOCKADDR_IN *addr);
    void sendResponse(fragment *frame, SOCKADDR_IN addr);
    void sendReset(SOCKADDR_IN addr);
    void sendResetResponse(SOCKADDR_IN addr);
    void resetCom(SOCKADDR_IN addr);
    uint32_t getNextDataLength();
    static SOCKADDR_IN getAddr(string ip, int port);
#ifdef RESEND_COUNT
    void show() {
        cout << resendCount << endl;
    }
#endif

    friend unsigned __stdcall recvThread(LPVOID data);
    friend unsigned __stdcall sendDataThread(LPVOID data);
#ifdef CHECK_SUM
    friend uint32_t calcCheckSum(ReliUDP *godFather, fragment *frame, int size);
#endif

private:
    int messageSeqID;
    SOCKADDR_IN localAddr;
    int localPort;
    string localIP;
    SOCKET sock;
    sendStat ST;
    recvBuffer BUF;
#ifdef CHECK_SUM
    CRC32 crcObj;
#endif
    bool stat;
    bool dataCopyingFlag;
    bool resetWaitFlag;
    HANDLE recvThreadHandle;
    CRITICAL_SECTION sendMutex;
    CRITICAL_SECTION sendStatMutex;
    CRITICAL_SECTION bufMutex;
    CRITICAL_SECTION sendCountMutex;
    CRITICAL_SECTION messageSeqIdMutex;
    CRITICAL_SECTION threadNumMutex;
#ifdef RESEND_COUNT
    CRITICAL_SECTION resendCountMutex;
    uint16_t resendCount;
#endif
    uint16_t sendCount;
    uint16_t threadNum;
    clock_t lastSendTime;
};

typedef struct _sendThreadPara {
    _sendThreadPara(ReliUDP *father, const char *d, uint32_t len, uint32_t id, SOCKADDR_IN add, char opt) {
        godFather = father;
        dat = d;
        dataLength = len;
        messageSeqId = id;
        sendOpt = opt;
        addrCopy(&addr, add);
    }
    ReliUDP *godFather;
    const char *dat;
    uint32_t dataLength;
    uint16_t messageSeqId;
    SOCKADDR_IN addr;
    char sendOpt;
} sendPara;

