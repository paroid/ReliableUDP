#include "ReliUDP.h"

inline void waitForGodFather(HANDLE mutex){
	DWORD res;
#ifdef MUTEX_TIMEOUT
	clock_t timeoutTime=clock()+MUTEX_WAIT_TIMEOUT;
#endif
	do{
#ifdef MUTEX_TIMEOUT
		if(clock()>timeoutTime){
			cout<<"[Wait Timeout]"<<endl;
			return;
		}
#endif
		res=WaitForSingleObject(mutex,INFINITE);
	}while(res!=WAIT_OBJECT_0);	
}

#ifdef CHECK_SUM
inline uint32_t calcCheckSum(ReliUDP *godFather,fragment *frame,int size){
	frame->checkSum=0;
	uint32_t ckcSum=godFather->crcObj.check(frame,size);
	frame->checkSum=ckcSum;
	return ckcSum;
}
#endif

ReliUDP::ReliUDP(void)
{
	winSockInit();
	memset(&localAddr,0,sizeof(localAddr));
	memset(&remoteAddr,0,sizeof(remoteAddr));
	messageSeqID=0;
	stat=false;
	sendCount=0;
	lastSendTime=0;
	threadNum=0;
#ifdef RESEND_COUNT
	resendCount=0;
#endif
}


ReliUDP::~ReliUDP(void)
{
}


bool ReliUDP::winSockInit(){
	WORD wVersionRequested;  
	WSADATA wsaData;  	  

	wVersionRequested = MAKEWORD( 2, 2 );  
	int err = WSAStartup( wVersionRequested, &wsaData );  

	if ( err != 0 ) //winSock Error
		cout<<"WSAStartup Fail"<<endl;
	return SOCK_INIT_FAIL;  

	if ( LOBYTE( wsaData.wVersion ) != 2 || HIBYTE( wsaData.wVersion ) != 2) {  //version check
		WSACleanup( );  
		cout<<"WSAStartup WORD Fail"<<endl;
		return SOCK_INIT_FAIL;   
	}  
	return SOCK_INIT_OK;
}

void ReliUDP::setLocalPort(int port){	
	localPort=port;
}

void ReliUDP::setRemoteAddr(string ip,int port){	
	remoteIP=ip;
	remotePort=port;
}

void ReliUDP::setTempRemoteAddr(string ip,int port){
	remoteAddr.sin_family=AF_INET;
	remoteAddr.sin_port=htons(port);
	remoteAddr.sin_addr.S_un.S_addr=inet_addr(ip.data());
}


void ReliUDP::startCom(){
	if(stat)	//already started
		return;
	//local
	localAddr.sin_family=AF_INET;
	localAddr.sin_port=htons(localPort);
	localAddr.sin_addr.S_un.S_addr=htonl(INADDR_ANY);
	//remote	
	remoteAddr.sin_family=AF_INET;
	remoteAddr.sin_port=htons(remotePort);
	remoteAddr.sin_addr.S_un.S_addr=inet_addr(remoteIP.data());

	if((sock=socket(AF_INET,SOCK_DGRAM,0))<0)
		std::cout<<"Socket Error!"<<endl;
	if(bind(sock,(sockaddr *) &localAddr,sizeof(sockaddr))<0)
		std::cout<<"Bind Error!"<<endl;
	//create Mutex

	sendMutex=CreateMutex(NULL,FALSE,NULL);
	sendStatMutex=CreateMutex(NULL,FALSE,NULL);
	bufMutex=CreateMutex(NULL,FALSE,NULL);
	sendCountMutex=CreateMutex(NULL,FALSE,NULL);
	messageSeqIdMutex=CreateMutex(NULL,FALSE,NULL);
	threadNumMutex=CreateMutex(NULL,FALSE,NULL);
#ifdef RESEND_COUNT
	resendCountMutex=CreateMutex(NULL,FALSE,NULL);
#endif
	if(sendMutex || sendStatMutex || bufMutex || sendCountMutex || messageSeqIdMutex || threadNumMutex)  
	{  
		if(ERROR_ALREADY_EXISTS==GetLastError())  
		{  
			cout<<"Mutex Error"<<endl;  
			return;  
		}  
	}  

	//start recvThread
	stat=true;
	unsigned dwThreadID;
	recvThreadHandle=(HANDLE)_beginthreadex(NULL, 0, &recvThread, (LPVOID) this, 0, &dwThreadID);

}

void ReliUDP::stopCom(){
	//wait for send thread
	while(sendCount)
		Sleep(20);
	stat=false; 	
	char sign[9]="shutdown";
	setTempRemoteAddr("127.0.0.1",localPort);
	//sleep !important
	Sleep(50);
	udpSendData(sign,9);
	//wait for thread to terminate
	waitForGodFather(recvThreadHandle);
	CloseHandle(recvThreadHandle);
	//close mutex
	CloseHandle(sendMutex);
	CloseHandle(sendStatMutex);
	CloseHandle(sendCountMutex);
	CloseHandle(bufMutex);
	CloseHandle(messageSeqIdMutex);
	CloseHandle(threadNumMutex);
#ifdef RESEND_COUNT
	CloseHandle(resendCountMutex);
#endif
	closesocket(sock);  	
}

void ReliUDP::clearCom(){
	stopCom();
	WSACleanup();
}

void ReliUDP::udpSendData(const char *dat,int dataLength){
	if(sendto(sock,dat,dataLength,0,(sockaddr *) &remoteAddr,sizeof(sockaddr))<0)
		std::cout<<"Send Error: "<<WSAGetLastError()<<endl;
}

int ReliUDP::udpRecvData(char *buf,int dataLength){
	int res=sizeof(sockaddr);
	if((res=recvfrom(sock,buf,dataLength,0,(sockaddr *)&remoteAddr,&res))<0)
		std::cout<<"Recv Error: "<<WSAGetLastError()<<endl;
	return res;
}


void ReliUDP::sendData(const char *dat,int dataLength,char sendOpt){	
	waitForGodFather(messageSeqIdMutex);
	//here inc messageSeqID
	++messageSeqID;
	ReleaseMutex(messageSeqIdMutex);

	waitForGodFather(sendCountMutex);
	++sendCount;
	ReleaseMutex(sendCountMutex);

	if((sendOpt&SEND_BLOCK_CHECK) == SEND_UNBLOCK){	//unblock send threadNum check
		waitForGodFather(threadNumMutex);
		if(threadNum>MaxThread)
			sendOpt|=SEND_BLOCK;
		else
			++threadNum;
		ReleaseMutex(threadNumMutex);
	}

	sendPara *para=new sendPara(this,dat,dataLength,messageSeqID,sendOpt);	

	if((sendOpt&SEND_BLOCK_CHECK) == SEND_BLOCK){	//block send
		sendDataThread(para); 
	}
	else{	//unblock send		
		waitForGodFather(sendMutex);
		dataCopyingFlag=true;
		unsigned dwThreadID;
		HANDLE hThread=(HANDLE)_beginthreadex(NULL, 0, &sendDataThread, (LPVOID) para, 0, &dwThreadID);
		CloseHandle(hThread);
		while(dataCopyingFlag)
			Sleep(5);
		ReleaseMutex(sendMutex);
	}
}


unsigned __stdcall sendDataThread(LPVOID data){
	//parameters prepare
	sendPara *para=(sendPara *)data;
	ReliUDP *godFather=para->godFather;
	int dataLength=para->dataLength;
	const char *dat;	
	if((para->sendOpt&SEND_BLOCK_CHECK) == SEND_BLOCK)	//block send 
		dat=para->dat;
	else{	//unblock send new mem
		dat=new char[dataLength];
		memcpy((void *)dat,para->dat,dataLength);
		//release mutex
		godFather->dataCopyingFlag=false;	
	}				

	int fragmentCount=dataLength/FragmentDataSize+((dataLength%FragmentDataSize) != 0);	

	//the sending frame
	fragment frame;
	frame.type=FRAGMENT_DATA;
	frame.messageSeqID=para->messageSeqId;	//note: this is the real messageSeqId godfather->messageSeqID maybe not !
	frame.fragmentNum=fragmentCount;
	frame.dataSize=dataLength;


	//wait for mutex
	waitForGodFather(godFather->sendStatMutex);
	//set the send stat
	godFather->ST.newSeq(&frame);
	godFather->ST.allSet(&frame);
	ReleaseMutex(godFather->sendStatMutex);


	//calculate timeout & sleep
	clock_t timeoutTime=clock()+SendTimeout+fragmentCount*SendTimeoutFactor;
	clock_t noReceiverTimeout=clock()+SendNoReceiverTimeout;	//caused by no valid receiver
	vector<int> queue;
	uint32_t prevSize=0;
	bool flag=false;
	uint32_t expectedSize=0;
	clock_t timer;
	while(1){
		queue.clear();	//let me make it clear

		waitForGodFather(godFather->sendStatMutex);
		//check the send stat	get all un-response frame 	
		queue=godFather->ST.getAll(&frame);

		bool over=false;
		if(queue.empty()){	//all clear
#ifdef DEBUG
			cout<<"[send complete: "<<frame.messageSeqID<<"]"<<endl;
#endif
			over=true;
		}

		//timoutCheck
		if(clock()>timeoutTime || (queue.size()==fragmentCount && clock()>noReceiverTimeout)){
#ifdef DEBUG
			cout<<"[Send Abort]"<<endl;
#endif
			over=true;
		}
		if(over){
			//release mutex before return
			ReleaseMutex(godFather->sendStatMutex);

			waitForGodFather(godFather->sendCountMutex);
			--godFather->sendCount;
			ReleaseMutex(godFather->sendCountMutex);
			if((para->sendOpt&SEND_BLOCK_CHECK) == SEND_UNBLOCK){
				delete[] dat; //unblock send -> free mem
				waitForGodFather(godFather->threadNumMutex);
				--godFather->threadNum;
				ReleaseMutex(godFather->threadNumMutex);
			}
			delete data;			
			return 0;
		}		
		ReleaseMutex(godFather->sendStatMutex);
		
		//responsive send
		expectedSize*=ExpectRate;
		flag=(queue.size()<ExpectExceptionSize || prevSize-queue.size()>=expectedSize || clock()>timer);
		prevSize=queue.size();
		if(!flag){	//do not resend
			Sleep(5); 
			continue; 
		}
		expectedSize=0;	
		//send frames
		int SendSampleInc=queue.size()/(SendSampleSize/FragmentSize+1)+1;	

		int lastDataSize=dataLength-(fragmentCount-1)*FragmentDataSize;
		int lastFrameSize=lastDataSize+FragmentHeaderSize;

		for(size_t i=0;i<queue.size();i+=SendSampleInc){
			++expectedSize;	
			frame.fragmentID=queue[i];

			int dataSize=FragmentDataSize;
			int frameSize=FragmentSize;
			if(queue[i] == fragmentCount-1){ //last one
				dataSize=lastDataSize;
				frameSize=lastFrameSize;
			}

			memcpy(frame.data,dat+queue[i]*FragmentDataSize,dataSize);
#ifdef CHECK_SUM
			//checkSum
			calcCheckSum(godFather,&frame,frameSize);
#endif
			godFather->udpSendData((const char *)&frame,frameSize);	
			Sleep(1);

#ifdef DEBUG_RS
			cout<<"[resend: -->>] "<<frame.messageSeqID<<"--"<<frame.fragmentID<<endl;
#endif
#ifdef RESEND_COUNT
			waitForGodFather(godFather->resendCountMutex);
			++godFather->resendCount;
			ReleaseMutex(godFather->resendCountMutex);
#endif
		}
		timer=clock()+ExpectTimeout;
		//wait for next check
		int sleepTime=TimeWait+int(TimeWaitSizeFactor*queue.size());	//sleep 
		Sleep(sleepTime);
	}	
}


void ReliUDP::sendResponse(fragment *frame){
	//the response frame
	fragment resFrame;
	resFrame.type=FRAGMENT_RESPONSE;
	resFrame.messageSeqID=frame->messageSeqID;
	resFrame.fragmentID=frame->fragmentID;	
	resFrame.dataSize=0;	//no data
#ifdef CHECK_SUM
	//checkSum
	calcCheckSum(this,&resFrame,FragmentHeaderSize);
#endif
	udpSendData((const char *)&resFrame,FragmentHeaderSize);
}

int getFrameLength(fragment *frame){
	switch(frame->type){
	case FRAGMENT_RESPONSE:
		return FragmentHeaderSize;		
	case FRAGMENT_DATA:
		return frame->fragmentID == frame->fragmentNum-1 ? FragmentHeaderSize+frame->dataSize-(frame->fragmentNum-1)*FragmentDataSize :FragmentSize;		
	default:
		frame->type=FRAGMENT_INVALID;
		return 0;
	}
}


unsigned __stdcall recvThread(LPVOID data){
	ReliUDP *godFather=(ReliUDP *)data;
	const int bufSize=FragmentSize;	//Data Content + Header
	char buf[bufSize];	
	int recvLen,dataLen;	
	fragment *frame;	//to form a frame
	recvStat RT;	//recv stat	

	while(godFather->stat){	//check service stat
		recvLen=godFather->udpRecvData(buf,bufSize);
		if(recvLen>=FragmentHeaderSize){	//basic requirement to be a ReliUDP frame
			frame=(fragment *)buf;		//let's recognize it as a frame

			int frameLen=getFrameLength(frame);		

#ifdef CHECK_SUM
			//checkSum
			uint32_t checkSum=frame->checkSum;
			uint32_t realCkcSum=calcCheckSum(godFather,frame,frameLen);

			if(checkSum != realCkcSum){
				frame->type=FRAGMENT_INVALID;	//mark as invalid fragment
			}
#endif
			switch(frame->type){
			case FRAGMENT_RESPONSE:				
#ifdef DEBUG_RS
				cout<<"[resp: <<--] "<<frame->messageSeqID<<"--"<<frame->fragmentID<<endl;
#endif

				waitForGodFather(godFather->sendStatMutex);
				//reset in send stat				
				godFather->ST.reset(frame);
				ReleaseMutex(godFather->sendStatMutex);

				break;
			case FRAGMENT_DATA:
#ifdef DEBUG_RS
				cout<<"[recv: ----] "<<frame->messageSeqID<<"--"<<frame->fragmentID<<endl;
#endif
				//whatever sender needs a response
				godFather->sendResponse(frame);

				//check duplicate message
				if(RT.checkSeqId(frame)){	//duplicate message
					//do nothing
					break;
				}

				if(!RT.have(frame->messageSeqID)){	//new entry
					RT.newSeq(frame);
				}
				//let's do it
				dataLen=frameLen-FragmentHeaderSize;

				if(RT.check(frame)){ //make sure this frame hasn't been received
					RT.storeData(frame,dataLen);	//storeData & set Bit

					//check if all received
					if(RT.getOne(frame->messageSeqID)==-1){	//all received
						//add messageSeqID
						RT.addSeqId(frame);

						waitForGodFather(godFather->bufMutex);						
						godFather->BUF.addEntry(RT.getDataPointer(frame->messageSeqID),frame->dataSize);						
						ReleaseMutex(godFather->bufMutex);

						RT.removeSeq(frame->messageSeqID);
#ifdef DEBUG
						cout<<"[ID:"<<frame->messageSeqID<<" received]"<<endl;
#endif
					}
				}

				//check recv stat to remove timeout entries
				RT.removeTimeout();
				break;
			case FRAGMENT_INVALID:	//do nothing
#ifdef DEBUG
				cout<<"[invalid frame]"<<endl;
#endif
				break;
			}
		}
	}	
	return 0;
}


int ReliUDP::recvData(char *buf,int dataLength){
	while(BUF.empty())
		Sleep(RecvBUFWait);
	waitForGodFather(bufMutex);	
	int ret=BUF.popEntry(buf);
	ReleaseMutex(bufMutex);
	return ret;
}

uint32_t ReliUDP::getNextDataLength(){
	return BUF.getHeadSize();
}
