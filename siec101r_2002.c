#include "siec101_2002.h"

//=============================================================
// 名称: SIEC101r_2002.c
// 功能: 处理S101规约的接收报文
// 编写：盖希波2003.12.10  
// 改写：2019.6.4
//==============================================================
extern uint8 UserFlag[10];   //控制域功能码
extern TIEC101Cfg_Handle m_pCfg;  //报文配置句柄
extern TxdFrame_buf m_Txd;    //固定帧结构体
RxdFrame_buf m_RxdFrame ; //接收报文缓冲区
uint8 singleYx[YX_LENTH]={0};  //遥信数据
uint8 singleYc[YC_LENTH*2]={0};  //遥测数据

/**
 * @description: 接收处理
 * @param {none} 
 * @return:处理结果的状态，TRUE处理数据成功，FALSE条件不满足，未处理数据
 */
FuncStatus RxdMonitor()
{
    uint8 *pBuf;
    if (SearchFrame()!=TRUE){ //检索报文是否有效
		perror("Searching frame error!");
        return FALSE;
	}
    pBuf= m_RxdFrame.buf;
    
    perror("finishing RxdMonitor!");
	if(pBuf[0] == 0x10)
		return RxdFixFrame();  //固定帧结构处理 
	
	if(pBuf[0] == 0x68)
		return RxdVarFrame(); //可变帧长结构处理   

	return FALSE;
}

/**
 * @description:判断是否有效报文 
 * @param {none} 
 * @return:函数状态，返回TRUE表示报文有效，返回FALSE表示报文错误
 */
FuncStatus SearchFrame()  
{
	uint8 *pBuf=m_RxdFrame.buf;   //获取报文
    uint16 wFrameLen;
	uint8 bChkSum;
	uint8 bChkLen;
	uint16 wAddr;
	switch(pBuf[0])  //报文结构选择 
	{
	case 0x10:  //固定帧结构 
		perror("This is a fixframe");
		if(m_pCfg.bLinkAddr == 2)//链路层地址长度2
		{
			perror("m_pCfg.bLinkAddr lenth =2 ！");
			bChkLen = 3;//校验长度3
			wFrameLen = 6; //固定帧长度为6，1个字节启动字符，1个字节链路控制，2个字节链路地址，1个字节校验和，1个字节结束字符 
	
			if(pBuf[5] != 0x16){//判断结束符是否正确 
				perror("End charater error !");
				return FALSE;	 
			}
			wAddr = MakeAddr(pBuf[2], pBuf[3]); //获取两个字节的链路地址 

			if(SwitchToAddress(wAddr) != TRUE){//无效地址 
				perror("Invalid linkAddress !");
				return FALSE;
			}

			bChkSum = ChkSum(pBuf+1, bChkLen); //求检验和 
		
			if(pBuf[4] != bChkSum){  //判断检验和是否正确
				perror("check sum error !");
				return FALSE;	 
			}

			return TRUE; //报文正确
		}
		else//链路层地址长度1
		{
			bChkLen = 2;  //校验长度2
			wFrameLen = 5; //帧长度5
			perror("m_pCfg.bLinkAddr lenth=1 !");
			if(pBuf[4] != 0x16){
				perror("End charater error!");
				return FALSE;	 
			}

			wAddr = (uint16)pBuf[2];

			if(SwitchToAddress(wAddr) != TRUE){  //无效地址
				perror("invalid linkAddress");
				return FALSE;
			}

			bChkSum = ChkSum(pBuf+1, bChkLen); 
	
			if(pBuf[3] != bChkSum){ //校验和错误
				perror("check sum arise error");
				return FALSE;	 
			} 

			return TRUE;
		}
	case 0x68:  //可变帧长帧结构 ，固定4字节报文头 
		perror("This is a varframe");
		if(pBuf[1] != pBuf[2]){ //两次发送数据帧长度字节不相等，则返回错误 
			perror("received frame lenth error ！");
			return FALSE;
		 }
		 if(pBuf[3] != 0x68){  //再次判断启动字符 
		 	perror("The second start charater error !");
			return FALSE;
		 }
		 wFrameLen = pBuf[1] + 6; //可变帧的总长度=长度域+6个固定报文头报文尾 

		 if(pBuf[wFrameLen-1] != 0x16){ //判断帧结束字符 
		 	perror("End charater error !");
			return FALSE;
		 }
		 bChkSum = ChkSum(pBuf+4, pBuf[1]); //获取校验和 
	
		 if(pBuf[wFrameLen-2] != bChkSum){ //判断校验是否正确; 
		 	perror("check sum error !");
			return FALSE;
		 }
		 if(m_pCfg.bLinkAddr == 2)//链路层地址长度为2
			 wAddr = MakeAddr(pBuf[5], pBuf[6]); //获取链路地址 
		 else
			 wAddr = (uint16)pBuf[5];

		 if(SwitchToAddress(wAddr) != TRUE){//无效地址
		 	perror("invalid linkAddress");
			 return FALSE;
		 }
		 return TRUE;

	default:
		perror("not fixframe or varframe");
		return FALSE;
	}	
}

/**
 * @description: 固定帧或可变帧的链路地址为两个字节时，将地址转换为16位地址
 * @param {低字节链路地址，高字节链路地址} 
 * @return:转换后的16位地址
 */
uint16 MakeAddr(uint8 low_linAddr,uint8 high_linAddr)
{
	uint16 bLinAddr;
	bLinAddr=high_linAddr;
	bLinAddr<<=8;           //高位链路地址
	bLinAddr+=bLinAddr;     //链路地址=高位链路地址+低位链路地址
	return bLinAddr;
}

/**
 * @description:计算帧的校验和值
 * @param {校验和变量的起始地址，检验和变量的长度} 
 * @return: 校验和值
 */
uint8 ChkSum(uint8* p_Addr,uint8 chek_len)
{
	uint8 i,checkSum=0;  
	uint8* headAddr;
	headAddr=p_Addr;
	for(i=0;i<chek_len;i++){
		checkSum+=*headAddr;     //计算校验和，不考虑溢出
	//	perror("*checkSum=%x\n",checkSum);
		headAddr++;
	}
	return checkSum;
}

/**
 * @description:检验本地地址与报文的地址是否相同（链路地址和公共地址） 
 * @param {报文链路地址} 
 * @return: RUTE：链路地址正确；FALSE：链路地址错误
 */
FuncStatus SwitchToAddress(uint16 LinkAddr)
{
	if((LinkAddr==m_pCfg.wProtocolID)||(LinkAddr==m_pCfg.bLinkAddr))  
		return TRUE;
	else
		return FALSE;
}


/**
 * @description:固定帧结构处理 
 * @param {none} 
 * @return: 状态量，TRUE处理数据成功，FALSE条件不满足，不处理
 */
FuncStatus RxdFixFrame()
{
	uint8* pBuf;
	uint8  bControl;

	pBuf = m_RxdFrame.buf;
	bControl = pBuf[1];

	Save_CtrlCode(SF_RXDCONTROL, bControl);

	if(m_pCfg.bTestMode == 0) { //1测试模式，0正常模式
		if(CheckAndRetrans(bControl))  //检验对方是否收到报文
			return TRUE;
	} 

	switch(bControl & BIT_FUNC)
	{
		case 0x00: //0x00 复位远方链路
			RxdResetLink();  
			perror("Reset remote link");
			break; 
		case 0x08://相应,链路状态
		case 0x09: //0x09 请求远方链路状态
			RxdReqLinkStatus(); 
			perror("Request remote link status");
			break; 
		case 0x0A: //召唤一级用户数据
			RxdClass1Data(); 
			perror("Calling one-level user data");
			break; 
		case 0x0B: //远方链路状态完好或召唤二级用户数据
			RxdClass2Data();
			perror("Calling second-level user data"); 
			break; 
		default:
			Fix_ACK();
			break;
		}
		return TRUE;
}

/**
 * @description:可变帧结构处理 
 * @param {none} 
 * @return:状态量，TRUE
 */
FuncStatus RxdVarFrame() 
{
	uint8* pBuf;
	uint8 bControl;
	uint8 bType;  //类型标识符
	uint16 wAddress;
	uint8* pData;

	pBuf = m_RxdFrame.buf;
	bControl = pBuf[4]; //链路控制域 

	Save_CtrlCode(SF_RXDCONTROL, bControl);

	if(m_pCfg.bTestMode == 0)
	{
		if(*SF_RtuInitok == FALSE) //没有初始化链路
			return FALSE;

		if(CheckAndRetrans(bControl))//重发数据
			return FALSE;
	}

	if(!ChkVarControl(bControl&0x0F))  //检验链路是否完好
		return FALSE;

	pData = pBuf+9;  //应用服务单元公共地址，默认前面属性1字节
	
	if(m_pCfg.bLinkAddr == 2)  //链路地址2字节
	{
		bType = pBuf[7];  
		pData++;
	}
	else
		bType = pBuf[6];
	
	if(m_pCfg.bCauseTransLen == 2) //传输原因2字节
		pData ++;
	if(m_pCfg.bAsduAddrLen == 2) //公共地址2字节
		wAddress = MakeAddr(pData[0],pData[1]); 
	else
		wAddress = pData[0];
	
	if(SwitchToAddress(wAddress) != TRUE)//无效地址
		return FALSE;

	switch(bType)
	{
	case 0x64:  //总召唤
		RxdCallAll(); 
		perror("Calling all");
		break;
	case 0x67:  //时钟同步
		//RxdClockSyn(); 
		break;
	case 0x68:  //测试链路
		//RxdTestLink();
		break;
	default:
		//RxdNoAsdu();
		break;
	}
	return TRUE;
}

/**
 * @description: 0x00 复位远方链路
 * @param {none} 
 * @return: 
 */
FuncStatus RxdResetLink()
{
	uint8 *pBuf;
	uint8 bControl;
	pBuf = m_RxdFrame.buf;
	bControl = pBuf[1];
	if(bControl & BIT_PRM){
		*SF_RtuInitok=1; //生成初始化结束事件
	}
	Fix_ACK();  
	return TRUE;
}

/**
 * @description:请求远方链路状态 
 * @param {none} 
 * @return:函数状态，TRUE为真
 */
FuncStatus RxdReqLinkStatus()
{
	TxdFixFrame(0, 0x0B);
	return TRUE;
}

/**
 * @description:召唤一级用户数据 
 * @param {none} 
 * @return: 
 */
FuncStatus RxdClass1Data()
{
	if(*SF_RespCall_Flag==1){
		*SF_RespCall_Flag=0;
		Txd_CallAll_Respond();  //总召确认
		return TRUE;
	}

	if(*SF_Class1YX_Flag==1){  //遥信
		*SF_Class1YX_Flag=0;
		Txd_YXData(YX_LENTH,0x0001,singleYx);  //总召YX上送
		return TRUE;
	}

	if(*SF_Class1YC_Flag==1){  //遥测
		*SF_Class1YC_Flag=0;
		TXd_YCData(YC_LENTH,0x4001,singleYc,1);  //总召YC上送
		return TRUE;
	}
	if(*SF_AllCallEnd_Flag==1){
		*SF_AllCallEnd_Flag=0;

		Txd_CallAllEnd(); //总召结束
		return TRUE;
	}
	

	if(Txd_Class1())
		return TRUE;
		
	Txd_NoData();
//	Txd_NoData_E5();

	return TRUE;
}

/**
 * @description:召唤二级用户数据 
 * @param {none} 
 * @return: 状态量
 */
FuncStatus RxdClass2Data()//远方链路状态完好或召唤二级用户数据
{
	if(Txd_Class2())//上送二级用户数据
		return TRUE;
	
	if(Txd_Class1())//上送一级用户数据
		return TRUE;

	Txd_NoData();//无用户数据
//	Txd_NoData_E5();

	return TRUE;
}

/**
 * @description:总召唤
 * @param {none} 
 * @return: TRUE
 */
void RxdCallAll()
{
//	Txd_CallAll_Respond();  //总召确认
//	TXd_YCData(YC_LENTH|0x80,0x4001,singleYc,1);  //总召YC上送
//	Txd_YXData(YX_LENTH|0x80,0x0001,singleYx);  //总召YX上送
//	Txd_CallAllEnd(); //总召结束
//	SetALLDataFlag();
	*SF_RespCall_Flag=1;//总召命令确认标志位
	*SF_Class1YX_Flag=1;//总召时，终端所有有效数据生成一级用户数据事件
	*SF_Class1YC_Flag=1;
	*SF_AllCallEnd_Flag=1; //总召命令结束标志位

	Fix_ACK(); 
}

/**
 * @description:检验链路是否完好
 * @param {funCode控制域的功能码} 
 * @return:函数状态， RUTE：链路完好 FAULE：链路忙或链路服务未工作
 */
FuncStatus ChkVarControl(uint8 funCode) 
{
	uint8 funCodeTemp;

	funCodeTemp = funCode&0x0F;
	
	if(funCodeTemp == 4)//no answer
		return FALSE;
	if(funCodeTemp == 3) //确认用户数据
		return TRUE;

	TxdFixFrame(0, 15);//服务未完成

	return FALSE;
}

/**
 * @description:检验对方是否正确接收报文 
 * @param {bControl:控制字} 
 * @return: FALSE：不需要重发，TRUE重发数据
 */
FuncStatus CheckAndRetrans(uint8 bControl)
{
	uint8 funcCode;
	uint8 bNextFcbFcv;
	uint8 bRxdFcbFcv;

	funcCode = bControl&BIT_FUNC;
	bNextFcbFcv = *SF_NEXT_FCBFCV & BIT_FCBFCV;  //预期FCBFCV
	bRxdFcbFcv = bControl&BIT_FCBFCV;

	if(funcCode==0||funcCode==1)//复位帧计数，期待下一个FCB=1,FCV=1；
	{
		*SF_NEXT_FCBFCV=BIT_FCBFCV;
		return FALSE;
	}
	

	if(!(bRxdFcbFcv&BIT_FCV))//本次FCV无效,不需要重发数据
	{
		*SF_NEXT_FCBFCV=0X00;
		return FALSE;
	}
		
	if(bNextFcbFcv == 0) //上一帧是不需要重发的帧
	{
		*SF_NEXT_FCBFCV=(~bRxdFcbFcv&BIT_FCB)|BIT_FCV;
		return FALSE;
	}

	if(bNextFcbFcv != bRxdFcbFcv)  //实际FCBFCV与预期的不一样，重发
	{
		if(m_Txd.WritePtr > 4)   //判断报文是否正确
		{
			TxdRetry();  //重发上一次数据帧
			return TRUE;
		}
	}
	*SF_NEXT_FCBFCV=(~bRxdFcbFcv&BIT_FCB)|BIT_FCV; //下次接收的FCB位翻转，FCV位置1
	return FALSE;
}

