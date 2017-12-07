#ifndef CANMSG_INCLUDEDEF_H
#define CANMSG_INCLUDEDEF_H
//-----------------------------------------------
#include <iostream>
#include <iomanip>
#include "cm_can_id.h"

using namespace std;
//-----------------------------------------------

namespace can {
/**
 * Represents a CAN message.
 * \ingroup DriversCanModul
 */
class CanMsg
{
public:
	/// Include typedefs from windows.h
	typedef unsigned char BYTE;
	/// @todo This should be private.
	int m_iID;
	/// @todo This should be private.
	int m_iLen;
	/// @todo This should be private.
	int m_iType;

protected:
	/**
	 * A CAN message consists of eight bytes.
	 */
	BYTE m_bDat[8];

public:
	/**
	 * Default constructor.
	 */
	CanMsg()
	{
		m_iID = 0;
		m_iLen = 8;
		m_iType = 0x00;
        set(0,0,0,0,0,0,0,0);
	}

	/**
	 * Sets the bytes to the telegram.
	 */
	void set(BYTE Data0=0, BYTE Data1=0, BYTE Data2=0, BYTE Data3=0, BYTE Data4=0, BYTE Data5=0, BYTE Data6=0, BYTE Data7=0)
	{
		m_bDat[0] = Data0;
		m_bDat[1] = Data1;
		m_bDat[2] = Data2;
		m_bDat[3] = Data3;
		m_bDat[4] = Data4;
		m_bDat[5] = Data5;
		m_bDat[6] = Data6;
		m_bDat[7] = Data7;
	}

	/**
	 * Set the byte at the given position.
	 */
	void setAt(BYTE data, int iNr)
	{
		m_bDat[iNr] = data;
	}

	/**
	 * Gets the bytes of the telegram.
	 */
	void get(BYTE* pData0, BYTE* pData1, BYTE* pData2, BYTE* pData3, BYTE* pData4, BYTE* pData5, BYTE* pData6, BYTE* pData7)
	{
		*pData0 = m_bDat[0];
		*pData1 = m_bDat[1];
		*pData2 = m_bDat[2];
		*pData3 = m_bDat[3];
		*pData4 = m_bDat[4];
		*pData5 = m_bDat[5];
		*pData6 = m_bDat[6];
		*pData7 = m_bDat[7];
	}

	/**
	 * Returns a spezific byte of the telegram.
	 * @param iNr number of the byte.
	 */
	int getAt(int iNr)
	{
		return m_bDat[iNr];
	}

	/**
	 * Prints the telegram to the standard output.
	 * @deprecated function uses a spetific format of the telegram.
	 */
	int printCanIdentMsgStatus()
	{
		if(getStatus() == 0)
		{
			std::cout << "ID= " << m_iID << "  " << "Cmd= " << getCmd() << "   " << "Msg_OK" << std::endl;
			return 0;
		}
		else
		{
			std::cout << "ID= " << m_iID << "  " << "Cmd= " << getCmd() << "   " << "Msg_Error" << std::endl;
			return -1;
		}
	}

	/**
	 * Prints the telegram verbose.
	 */
	void print_v()
	{
        can_id_union can_id_u;
        can_id_u.can_id = m_iID;

		std::cout << "SrcMacID = 0x" << std::hex << std::setw(2) << std::setfill('0') << can_id_u.can_id_stru.SrcMacID << " DstMacID = 0x" << std::hex << std::setw(2) << std::setfill('0') << can_id_u.can_id_stru.DstMacID << " Ack = " << std::dec << can_id_u.can_id_stru.Ack << " FuncID = 0x" << std::hex << std::setw(1) << can_id_u.can_id_stru.FuncID << " SourceID = 0x" << std::hex << std::setw(2) << std::setfill('0') << can_id_u.can_id_stru.SourceID << " len = " << m_iLen << " data = " << std::setw(2) << std::setfill('0') << (int)m_bDat[0] << " " << std::setw(2) << std::setfill('0') <<(int)m_bDat[1] << " " << std::setw(2) << std::setfill('0') <<(int)m_bDat[2] << " " << std::setw(2) << std::setfill('0') <<(int)m_bDat[3] << " " << std::setw(2) << std::setfill('0') <<(int)m_bDat[4] << " " << std::setw(2) << std::setfill('0') <<(int)m_bDat[5] << " " << std::setw(2) << std::setfill('0') <<(int)m_bDat[6] << " " << std::setw(2) << std::setfill('0') <<(int)m_bDat[7] << std::endl;
	}
	/**
	 * Prints the telegram.
	 */
	void print()
	{
		std::cout << "id = 0x" << std::hex << std::setw(8) << std::setfill('0') << m_iID << " len = " << m_iLen << " data = " << std::setw(2) << std::setfill('0') << (int)m_bDat[0] << " " << std::setw(2) << std::setfill('0') <<(int)m_bDat[1] << " " << std::setw(2) << std::setfill('0') <<(int)m_bDat[2] << " " << std::setw(2) << std::setfill('0') <<(int)m_bDat[3] << " " <<
			std::setw(2) << std::setfill('0') <<(int)m_bDat[4] << " " << std::setw(2) << std::setfill('0') <<(int)m_bDat[5] << " " << std::setw(2) << std::setfill('0') <<(int)m_bDat[6] << " " << std::setw(2) << std::setfill('0') <<(int)m_bDat[7] << std::endl;
	}

	/**
	 * @deprecated function uses a spetific format of the telegram.
	 */
	int getStatus()
	{
		//bit 0 and bit 1 contain MsgStatus
		return (int)(m_bDat[0] >> 6);
	}

	/**
	 * @deprecated function uses a spetific format of the telegram.
	 */
	int getCmd()
	{
		return (int)(m_bDat[0] & 0x003f);
	}

	/**
	 * Get the identifier stored in this message structure.
	 * @return the message identifier.
	 */
	int getID()
	{
		return m_iID;
	}

	void setID(int id)
	{
		if( (0 <= id) && (id <= 536870911) )
			m_iID = id;
	}

	/**
	 * Get the message length set within this data structure.
	 * @return The message length in the range [0..8].
	 */
	int getLength()
	{
		return m_iLen;
	}

	/**
	 * Set the message length within this message structure.
	 * @param len The message length. Its value must be in the range [0..8].
	 */
	void setLength(int len)
	{
		if( (0 <= len) && (len <= 8) )
			m_iLen = len;
	}

	/**
	 * Get the message type. By default, the type is 0x00.
	 * @return The message type.
	 */
	int getType()
	{
		return m_iType;
	}

	/**
	 * Set the message type. By default, the type is 0x00.
	 * @param type The message type.
	 */
	void setType(int type)
	{
		m_iType = type;
	}


};

} //namespace can
//-----------------------------------------------
#endif
