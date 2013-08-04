#pragma once

#include <iostream>
#include <WinSock2.h>
#include <process.h>
#include <list>
#include <vector>
#include <queue>
#include <time.h>
#include <Windows.h>

#define CHECK_SUM

typedef unsigned char uint8_t;			//8-bit
typedef unsigned short uint16_t;		//16-bit
typedef unsigned int uint32_t;			//32-bit

static const int MaxThread = 12;				//default 12
static const int recvThreadNum = 3;				//default 3

static const int OSBufferSize = 65536;			//OS bufferSize default 64k

static const int FragmentDataSize = 4096;		//default 4k
static const int FragmentHeaderSize = 12;		//header 12
static const int responseSize = 6;
static const int resetSize = 2;
static const int minPacketSize = resetSize;
static const int FragmentSize = FragmentHeaderSize + FragmentDataSize;

static const int TimeWait = 8 ;					//default 8
static const int TimeWaitSizeFactor = int((0.002 * double(FragmentDataSize) / 10240.0)) ;	//(0.002*(double(FRAGMENT_DATA_SIZE)/10240.0))			ms/frame

static const int SendSampleSize = 1024 * 24;	//default 24k

static const int SendTimeout = 3000;			//default 3k ms
static const int SendTimeoutFactor = 50;		//default 50  ms/frame
static const int SendNoReceiverTimeout = int(1000 * double(CLOCKS_PER_SEC) / 1000.0);		//default 1 s

static const double ExpectRate = .99; 			//default 99%
static const int ExpectTimeout = 32;			//default 32 ms
static const int ExpectExceptionSize = 12;		//default 12

static const int RecvSeqIDBufferSize = 320;		//default 200

static const int RecvBUFWait = 50;				//default 5ms

static const int FragmentTimeout = 3000;		//default 3k ms
static const int FragmentTimeoutFactor = 2;		//default 2ms/frame

static const int selectTimeoutTime = 1;			//default 1s

#define FRAGMENT_DATA 73
#define FRAGMENT_RESPONSE 81
#define FRAGMENT_INVALID 59
#define FRAGMENT_RESET 43
#define FRAGMENT_RESET_RESPONSE 91


#define ID_ERROR -2

#define SOCK_INIT_FAIL false
#define SOCK_INIT_OK true

#define SEND_BLOCK 1
#define SEND_UNBLOCK 0
#define SEND_BLOCK_CHECK 1	//bit 0

using namespace std;


#pragma pack(push)
#pragma pack(4)
typedef struct _fragmentST {
    uint8_t type;					//offset:0
    uint8_t checkSum;				//offset:1
    uint16_t messageSeqID;			//offset:2
    uint16_t fragmentID;			//offset:4
    uint32_t dataSize;				//offset:8
    char data[FragmentDataSize];	//offset:12
} fragment;
#pragma pack(pop)

inline bool checkTimeout(DWORD timeoutTime) {
    DWORD now = GetTickCount();
    return (now > timeoutTime && now - timeoutTime < (ULONG_MAX >> 1)) || (now < timeoutTime && timeoutTime - now > (ULONG_MAX >> 1));
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




//host info: IP + port + seqID
class IPortSeq {
public:
    IPortSeq(const SOCKADDR_IN *addr, uint16_t seq) {
        IP = addr->sin_addr.S_un.S_addr;
        port = addr->sin_port;
        SeqID = seq;
    }
    friend bool operator==(const IPortSeq &a, const IPortSeq &b);
    uint32_t IP;
    uint16_t port;
    uint16_t SeqID;
};


class statEntry {
public:
    statEntry(const SOCKADDR_IN *addr, uint16_t i, int n): ips(addr, i) {
        bits = new bitSet(n);
    }
    ~statEntry() {
        delete bits;
    }
    IPortSeq ips;
    bitSet *bits;
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
    list<statEntry>::iterator find(const fragment *frame, const SOCKADDR_IN *addr) {
        IPortSeq tseq(addr, frame->messageSeqID);
        for(list<statEntry>::iterator it = indexSet.begin(); it != indexSet.end(); ++it) {
            if(tseq == it->ips)
                return it;
        }
        return indexSet.end();
    }
    void newSeq(const fragment *frame, const SOCKADDR_IN *addr) {
        list<statEntry>::iterator it = find(frame, addr);
        if(it == indexSet.end()) {
            int fragmentCount = frame->dataSize / FragmentDataSize + ((frame->dataSize % FragmentDataSize) != 0);
            statEntry *entry = new statEntry(addr, frame->messageSeqID, fragmentCount);
            indexSet.push_back(*entry);
        }
    }
    void set(const fragment *frame, const SOCKADDR_IN *addr) {
        list<statEntry>::iterator it = find(frame, addr);
        if(it != indexSet.end())
            it->bits->set(frame->fragmentID);
    }
    void reset(const fragment *frame, const SOCKADDR_IN *addr) {
        list<statEntry>::iterator it = find(frame, addr);
        if(it != indexSet.end())
            it->bits->reset(frame->fragmentID);
    }
    void allSet(const fragment *frame, const SOCKADDR_IN *addr) {
        list<statEntry>::iterator it = find(frame, addr);
        if(it != indexSet.end())
            it->bits->allSet();
    }
    void allReset(const fragment *frame, const SOCKADDR_IN *addr) {
        list<statEntry>::iterator it = find(frame, addr);
        if(it != indexSet.end())
            it->bits->allReset();
    }
    int getOne(const fragment *frame, const SOCKADDR_IN *addr) {
        list<statEntry>::iterator it = find(frame, addr);
        if(it != indexSet.end())
            return it->bits->getOne();
        return ID_ERROR;
    }
    vector<int> getAll(const fragment *frame, const SOCKADDR_IN *addr) {
        list<statEntry>::iterator it = find(frame, addr);
        if(it != indexSet.end())
            return it->bits->getAll();
        vector<int> empty;
        return empty;
    }
    void removeSeq(const fragment *frame, const SOCKADDR_IN *addr) {
        list<statEntry>::iterator it = find(frame, addr);
        if(it != indexSet.end()) {
            indexSet.erase(it);
        }
    }

    list<IPortSeq>::iterator findSeq(const SOCKADDR_IN *addr) {
        for(list<IPortSeq>::iterator it = seqIndex.begin(); it != seqIndex.end(); ++it) {
            if(it->IP == addr->sin_addr.S_un.S_addr && it->port == addr->sin_port)
                return it;
        }
        return seqIndex.end();
    }
    bool haveIPort(const SOCKADDR_IN *addr) {
        return findSeq(addr) != seqIndex.end();
    }
    void newIPortSeq(const SOCKADDR_IN *addr) {
        IPortSeq *entry = new IPortSeq(addr, 1);
        seqIndex.push_back(*entry);
    }

    uint32_t getIPortSeq(const SOCKADDR_IN *addr) {
        list<IPortSeq>::iterator it = findSeq(addr);
        if(it != seqIndex.end()) {
            return ++it->SeqID;
        }
        else {
            newIPortSeq(addr);
            return 1;
        }
    }

private:
    list<statEntry> indexSet;
    list<IPortSeq> seqIndex;
};



class recvEntry {
public:
    recvEntry(const SOCKADDR_IN *addr, uint16_t i, int n, int dataSize): ips(addr, i) {
        size = n;
        bits = new bitSet(n);
        bits->allSet();	//initial 0xFF
        data = new char[dataSize];
        timeoutTime = GetTickCount() + FragmentTimeout + FragmentTimeoutFactor * n;
    }
    ~recvEntry() {
        delete bits;
        delete[] data;
    }
    bool timeout(DWORD now) {
        return checkTimeout(timeoutTime);	//obvious timoutOut or overflow => timeout
    }
public:
    IPortSeq ips;
    bitSet *bits;
    char *data;
    DWORD timeoutTime;
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
    //for receive IPortSeq Set
    void addSeqId(const fragment *frame, const SOCKADDR_IN *addr) {
        IPortSeq *entry = new IPortSeq(addr, frame->messageSeqID);
        receivedSeqSet.push_back(*entry);
        if(receivedSeqSet.size() > RecvSeqIDBufferSize)	//buffer overflow remove head
            receivedSeqSet.erase(receivedSeqSet.begin());
    }
    bool checkSeqId(const fragment *frame, const SOCKADDR_IN *addr) {
        IPortSeq tseq(addr, frame->messageSeqID);
        for(list<IPortSeq>::iterator it = receivedSeqSet.begin(); it != receivedSeqSet.end(); ++it)
            if(tseq == *it)
                return true;
        return false;
    }
    //for recv Buffer Stat
    list<recvEntry>::iterator find(const fragment *frame, const SOCKADDR_IN *addr) {
        IPortSeq tseq(addr, frame->messageSeqID);
        for(list<recvEntry>::iterator it = indexSet.begin(); it != indexSet.end(); ++it) {
            if(it->ips == tseq)
                return it;
        }
        return indexSet.end();
    }
    bool have(const fragment *frame, const SOCKADDR_IN *addr) {
        return find(frame, addr) != indexSet.end();
    }

    void newSeq(const fragment *frame, const SOCKADDR_IN *addr) {
        list<recvEntry>::iterator it = find(frame, addr);
        if(it == indexSet.end()) {
            int fragmentCount = frame->dataSize / FragmentDataSize + ((frame->dataSize % FragmentDataSize) != 0);
            recvEntry *entry = new recvEntry(addr, frame->messageSeqID, fragmentCount, frame->dataSize);
            indexSet.push_back(*entry);
        }
    }
    bool check(const fragment *frame, const SOCKADDR_IN *addr) {
        list<recvEntry>::iterator it = find(frame, addr);
        if(it != indexSet.end()) {
            return it->bits->check(frame->fragmentID);
        }
        return false;
    }
    void storeData(const fragment *frame, const SOCKADDR_IN *addr, int dataLen) {
        list<recvEntry>::iterator it = find(frame, addr);
        if(it != indexSet.end()) {
            memcpy((it->data) + (frame->fragmentID * FragmentDataSize), frame->data, dataLen);
            it->bits->reset(frame->fragmentID);	//reset mark as received
        }
    }
    const char* getDataPointer(const fragment *frame, const SOCKADDR_IN *addr) {
        list<recvEntry>::iterator it = find(frame, addr);
        if(it != indexSet.end()) {
            return it->data;
        }
        return NULL;
    }
    int getOne(const fragment *frame, const SOCKADDR_IN *addr) {
        list<recvEntry>::iterator it = find(frame, addr);
        if(it != indexSet.end())
            return it->bits->getOne();
        return ID_ERROR;
    }
    void removeSeq(const fragment *frame, const SOCKADDR_IN *addr) {
        list<recvEntry>::iterator it = find(frame, addr);
        if(it != indexSet.end()) {
            indexSet.erase(it);
        }
    }
    //clear receive Set
    void clearSeqSet(const fragment *frame, const SOCKADDR_IN *addr) {
        list<IPortSeq>::iterator it = receivedSeqSet.begin();
        for(size_t i = 0; i < receivedSeqSet.size(); ++i) {
            if(it->IP == addr->sin_addr.S_un.S_addr && it->port == addr->sin_port) {
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
        DWORD now = GetTickCount();
        list<recvEntry>::iterator it = indexSet.begin();
        for(size_t i = 0; i < indexSet.size(); ++i) {
            if(it->timeout(now)) {
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
        addr = add;
    }
    int popData(char *des, SOCKADDR_IN *addr) {
        memcpy(des, data, dataLength);
        *addr = this->addr;
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
    uint8_t check(const void* data, size_t length, uint32_t previousCrc32 = 0) {
        uint32_t crc = ~previousCrc32;
        unsigned char* current = (unsigned char*) data;
        while(length--)
            crc = (crc >> 8) ^ crc32Lookup[(crc & 0xFF) ^ *current++];
        uint8_t res = crc;
        for(int i = 0; i < 3; ++i) {
            crc >>= 8;
            res ^= crc;
        }
        return res;
    }
private:
    uint32_t crc32Lookup[256];
};





class ReliUDP {
public:
    ReliUDP(void);
    ~ReliUDP(void);
    void setLocalAddr(string ip, int port);
    void startCom();
    void stopCom();
    void clearCom();
    void resetCom(SOCKADDR_IN addr);
    void sendData(const char *dat, int dataLength, SOCKADDR_IN addr, char sendOpt = SEND_BLOCK);
    int recvData(char *buf, int dataLength, SOCKADDR_IN *addr);
    uint32_t getNextDataLength();
    static SOCKADDR_IN getAddr(string ip, int port);

#ifdef RESEND_COUNT
    void show() {
        cout << sendTotalCount << " / " << resendCount << " = " << double(sendTotalCount) / double(resendCount) << endl;
    }
#endif

private:
    bool winSockInit();
    void udpSendData(const char *dat, int dataLength, SOCKADDR_IN addr);
    int udpRecvData(char *buf, int dataLength, SOCKADDR_IN *addr);
    void sendResponse(fragment *frame, SOCKADDR_IN addr);
    void sendReset(SOCKADDR_IN addr);
    void sendResetResponse(SOCKADDR_IN addr);

    friend unsigned __stdcall recvThread(LPVOID data);
    friend unsigned __stdcall sendDataThread(LPVOID data);
#ifdef CHECK_SUM
    friend uint32_t calcCheckSum(ReliUDP *godFather, fragment *frame, int size);
#endif

    SOCKADDR_IN localAddr;
    int localPort;
    string localIP;
    SOCKET sock;
    sendStat ST;
    recvStat RT;
    recvBuffer BUF;
#ifdef CHECK_SUM
    CRC32 crcObj;
#endif
    bool stat;
    bool dataCopyingFlag;
    bool resetWaitFlag;
    HANDLE recvThreadHandle[recvThreadNum];
    CRITICAL_SECTION sendMutex;
    CRITICAL_SECTION sendStatMutex;
    CRITICAL_SECTION bufMutex;
    CRITICAL_SECTION sendCountMutex;
    CRITICAL_SECTION messageSeqIdMutex;
    CRITICAL_SECTION threadNumMutex;
    CRITICAL_SECTION recvStatMutex;
#ifdef RESEND_COUNT
    CRITICAL_SECTION resendCountMutex;
    uint32_t resendCount;
    uint32_t sendTotalCount;
#endif
    uint16_t sendCount;
    uint16_t threadNum;
};

typedef struct _sendThreadPara {
    _sendThreadPara(ReliUDP *father, const char *d, uint32_t len, uint32_t id, SOCKADDR_IN add, char opt) {
        godFather = father;
        dat = d;
        dataLength = len;
        messageSeqId = id;
        sendOpt = opt;
        addr = add;
    }
    ReliUDP *godFather;
    const char *dat;
    uint32_t dataLength;
    uint16_t messageSeqId;
    SOCKADDR_IN addr;
    char sendOpt;
} sendPara;
