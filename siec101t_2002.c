#include "siec101_2002.h"

//===================================================================
// 名称: SIEC101_2002Txd.cpp
// 功能: 处理SIEC101_2002规约的发送报文
// 编写：盖希波 2003.12.10
// 改写：2019.6.20
//==================================================================
extern uint8 UserFlag[10];
extern TIEC101Cfg_Handle m_pCfg; //报文配置文件

TxdFrame_buf m_Txd; //发送报文缓存区

/**
 * @description: 发送固定帧报文
 * @param {bPRM:启动报文位.1主站启动/0从站启动,bCode:功能码} 
 * @return: 
 */
FuncStatus TxdFixFrame(uint8 bPRM, uint8 bCode)
{
	uint8 bChkSum;
	uint8 *pBuf;

	pBuf = m_Txd.buf; //发送缓存区
	m_Txd.WritePtr = 0;

	pBuf[m_Txd.WritePtr++] = 0x10;					  //启动字符
	pBuf[m_Txd.WritePtr++] = GetCtrCode(bPRM, bCode); //获取控制字

	if (m_pCfg.bLinkAddr == 2)
	{
		pBuf[m_Txd.WritePtr++] = (uint8)(m_pCfg.bLinkAddr & 0x00ff);		//链路地址域（子站站址）低位地址
		pBuf[m_Txd.WritePtr++] = (uint8)((m_pCfg.bLinkAddr & 0xff00) >> 8); //链路地址域（子站站址）高位地址

		bChkSum = ChkSum(pBuf + 1, 3); //获取校验和
	}
	else
	{
		pBuf[m_Txd.WritePtr++] = (uint8)(m_pCfg.bLinkAddr & 0x00ff); //链路地址域（子站站址）

		bChkSum = ChkSum(pBuf + 1, 2);
	}

	pBuf[m_Txd.WritePtr++] = bChkSum;
	pBuf[m_Txd.WritePtr++] = 0x16;

	perror("start Transmission");
	if (!StartTrans(m_Txd.buf, m_Txd.WritePtr))
		return FALSE; //m_Txd:信息起始地址，m_Txd.WritePtr-1:信息体长度
	perror("Transmite succeed!");
	return TRUE;
}

/**
 * @description: 可变帧报文头部
 * @param {bPRM:启动报文位，bCode:控制域功能码，bType:类型标识符，bReason:传输原因} 
 * @return: 
 */
FuncStatus Txd_Head(uint8 bPRM, uint8 bCode, uint8 bType, uint8 bReason)
{
	uint8 *pBuf;
	uint16 wAddr;

	pBuf = m_Txd.buf;

	wAddr = m_pCfg.bLinkAddr; //获取链路地址

	pBuf[0] = pBuf[3] = 0x68; //启动报文

	pBuf[4] = GetCtrCode(bPRM, bCode); //获取控制字

	m_Txd.WritePtr = 5;

	pBuf[m_Txd.WritePtr++] = (uint8)(wAddr & 0x00ff); //低位链路地址

	if (m_pCfg.bLinkAddr == 2)
		pBuf[m_Txd.WritePtr++] = (uint8)(wAddr >> 8); //高位链路地址

	pBuf[m_Txd.WritePtr++] = bType; //类型标识符

	m_Txd.WritePtr++;

	pBuf[m_Txd.WritePtr++] = bReason | m_pCfg.bTestMode; //传输原因

	if (m_pCfg.bCauseTransLen == 2)
		pBuf[m_Txd.WritePtr++] = 0;

	wAddr = m_pCfg.wProtocolID;						  //获取公共地址
	pBuf[m_Txd.WritePtr++] = (uint8)(wAddr & 0x00ff); //公共地址低位

	if (m_pCfg.bAsduAddrLen == 2)
		pBuf[m_Txd.WritePtr++] = (uint8)(wAddr >> 8); //公共地址高位

	return TRUE;
}

/**
 * @description:可变帧报文尾部 
 * @param {bNum:信息体元素长度，inforAddr:信息体地址,inforEle:信息体元素} 
 * @return: 
 */
FuncStatus Txd_Tail(uint8 bNum, uint16 inforAddr, uint8 *inforEle, uint8 yes_yc)
{
	uint8 i, bChkSum;
	uint8 *pBuf;

	pBuf = m_Txd.buf;

	if (m_pCfg.bLinkAddr == 2)
		pBuf[8] = bNum; //信息体元素长度
	else
		pBuf[7] = bNum;

	pBuf[m_Txd.WritePtr++] = (uint8)(inforAddr & 0x00ff); //信息体低位地址

	if (m_pCfg.bInfoAddrLen == 2)
		pBuf[m_Txd.WritePtr++] = (uint8)(inforAddr >> 8); //信息体高位地址

	if (yes_yc)
		bNum *= 2;

	for (i = 0; i < bNum; i++)
		pBuf[m_Txd.WritePtr++] = *inforEle++; //写入信息体元素

	pBuf[1] = pBuf[2] = (m_Txd.WritePtr - 4); //报文长度

	bChkSum = ChkSum(pBuf + 4, pBuf[1]); //检验和

	pBuf[m_Txd.WritePtr++] = bChkSum;
	pBuf[m_Txd.WritePtr++] = 0x16;

	if (StartTrans(m_Txd.buf, m_Txd.WritePtr))
		return FALSE; //发送报文，m_Txd:信息起始地址，m_Txd.WritePtr:信息体长度

	return TRUE;
}

/**
 * @description:设置控制字 
 * @param {bPRM:启动报文位，bCode:功能码} 
 * @return: 返回控制字
 */
uint8 GetCtrCode(uint8 bPRM, uint8 bCode)
{

	uint8 bCodeTmp = 0;

	bCodeTmp += bCode;

	if (bPRM) //启动报文位
		bCodeTmp |= 0x40;

	if(*SF_RtuInitok==1) //初始化成功后才判断是否有一级用户数据
	if (SearchClass1())   //是否有一级用户数据
		bCodeTmp |= 0x20; //变位遥信为1级数据

	return bCodeTmp;
}

/**
 * @description:固定帧，子站向主站回应 
 * @param {none} 
 * @return:TRUE回答正确
 */
FuncStatus Fix_ACK()
{
	uint8 bRxdCode;
	uint8 bTxdCode = 0;

	bRxdCode = *SF_RXDCONTROL & BIT_FUNC;

	if (bRxdCode == 4) //no answer
		return TRUE;

	switch (bRxdCode)
	{
	case 0:
	case 1:
	case 2:
	case 3:
		bTxdCode = 0;
		break;
	case 4:
		return TRUE;
	case 8:
	case 9:
		bTxdCode = 11;
		break;
	default:
		bTxdCode = 15; //链路未完成
		break;
	}
	TxdFixFrame(0, bTxdCode); //发送固定帧

	return TRUE;
}

/**
 * @description:子站回应无一级二级数据 
 * @param {none} 
 * @return: 
 */
FuncStatus Txd_NoData()
{
	return TxdFixFrame(0, 0x09);
} //无所请求数据帧

/**
 * @description: 子站回应无一级二级数据
 * @param {none} 
 * @return: 
 */
FuncStatus Txd_NoData_E5()
{
	m_Txd.WritePtr = 0;
	m_Txd.buf[m_Txd.WritePtr++] = 0xE5;
	if (StartTrans(m_Txd.buf, m_Txd.WritePtr))
		return FALSE; //m_Txd:信息起始地址，1:信息体长度
	return TRUE;
}


FuncStatus SearchClass1()
{
	if(*SF_Class1YX_Flag==1)
		return TRUE;

	if(*SF_Class1YC_Flag==1)
		return TRUE;
		
	return FALSE;
}

/**
 * @description:一级用户数据 
 * @param {type} 
 * @return: 
 */
FuncStatus Txd_Class1()
{
	// Txd_Head();
	// Txd_Tail();
	return TRUE;
}

/**
 * @description:二级用户数据 
 * @param {type} 
 * @return: 
 */
FuncStatus Txd_Class2()
{
	// Txd_Head();
	// Txd_Tail();
	return TRUE;
}

/**
 * @description:总召激活确认 
 * @param {*inforEle：信息体内容指针} 
 * @return: 
 */
void Txd_CallAll_Respond()
{
	uint8 inforEle = 0x14;
	Txd_Head(0, 0x08, 0x64, 0x07);	 //0x64总召命令，0x07总召激活确认
	Txd_Tail(1, 0x0000, &inforEle, 0); //信息体内容长度为1，信息体地址为0x0000，信息体内容指针,不是遥测信号
}

/**
 * @description:上送遥测数据 
 * @param {num:信息体长度，inforAddr:信息体地址，YCData:遥测数据} 
 * @return: 
 */
void TXd_YCData(uint8 num, uint16 inforAddr, uint8 *YCData, uint8 yes_yc)
{
	Txd_Head(0, 0x08, 0x64, 0x07); //0x64总召命令，0x07总召激活确认
	Txd_Tail(num, inforAddr, YCData, 1);
}

/**
 * @description:上送遥信数据 
 * @param {num:信息体长度，inforAddr:信息体地址，YXData:遥信数据} 
 * @return: 
 */
void Txd_YXData(uint8 num, uint16 inforAddr, uint8 *YXData)
{
	Txd_Head(0, 0x08, 0x64, 0x07); //0x64总召命令，0x07总召激活确认
	Txd_Tail(num, inforAddr, YXData, 0);
}

/**
 * @description:总召结束 
 * @param {none} 
 * @return: 
 */
void Txd_CallAllEnd()
{
	uint8 inforEle = 0x14;
	Txd_Head(0, 0x08, 0x64, 0x0A);	 //0x64总召命令，0x0A总召激活结束
	Txd_Tail(1, 0x0000, &inforEle, 0); //信息体内容长度为1，信息体地址为0x0000，信息体内容指针
}

/**
 * @description:主站没接收到数据，重新发送 
 * @param {none} 
 * @return: RTUE:发送成功，FALSE：发送失败
 */
FuncStatus TxdRetry()
{
	StartTrans(m_Txd.buf, m_Txd.WritePtr); //发送报文，m_Txd:信息起始地址，m_Txd.WritePtr:信息体长度
	return TRUE;
}

