#ifndef CANITF_INCLUDEDEF_H
#define CANITF_INCLUDEDEF_H
//-----------------------------------------------
#include <can_interface/CanMsg.h>
//-----------------------------------------------

// for types and baudrates see: https://github.com/ipa320/cob_robots/blob/hydro_dev/cob_hardware_config/raw3-5/config/base/CanCtrl.ini
#define CANITFTYPE_CAN_PEAK	0
#define CANITFTYPE_CAN_PEAK_USB	1
#define CANITFTYPE_CAN_ESD	2
#define CANITFTYPE_CAN_DUMMY	3
#define CANITFTYPE_CAN_BECKHOFF	4
#define CANITFTYPE_SOCKET_CAN 5
#define CANITFTYPE_CAN_VCI 6

#define CANITFBAUD_1M 	0x0
#define CANITFBAUD_500K	0x2
#define CANITFBAUD_250K 0x4
#define CANITFBAUD_125K	0x6
#define CANITFBAUD_50K	0x9
#define CANITFBAUD_20K	0xB
#define CANITFBAUD_10K	0xD

/**
 * General interface of the CAN bus.
 * \ingroup DriversCanModul
 */
class CanItf
{
public:
	enum CanItfType {
		CAN_PEAK = 0,
		CAN_PEAK_USB = 1,
		CAN_ESD = 2,
		CAN_DUMMY = 3,
		CAN_BECKHOFF = 4,
		CAN_SOCKETCAN = 5,
        CAN_VCI = 6
	};

	/**
	 * The destructor does not necessarily have to be overwritten.
	 * But it makes sense to close any resources like handles.
	 */
	virtual ~CanItf() {
	}

	/**
	 * Initializes the CAN bus and returns success.
	 */
	virtual bool init_ret() = 0;

	/**
	 * Initializes the CAN bus.
	 */
	virtual void init() = 0;

	/**
	 * Sends a CAN message.
	 * @param pCMsg CAN message
	 * @param bBlocking specifies whether send should be blocking or non-blocking
	 */
	virtual bool transmitMsg(can::CanMsg CMsg, bool bBlocking = true) = 0;

	/**
	 * Reads a CAN message.
	 * @return true if a message is available
	 */
	virtual bool receiveMsg(can::CanMsg* pCMsg) = 0;

	/**
	 * Reads a CAN message.
	 * The function blocks between the attempts.
	 * @param pCMsg CAN message
	 * @param iNrOfRetry number of retries
	 * @return true if a message is available
	 */
	virtual bool receiveMsgRetry(can::CanMsg* pCMsg, int iNrOfRetry) = 0;

	/**
	 * Reads a CAN message with timeout.
	 * @param pCMsg CAN message
	 * @param nMicroSecTimeout timeout in us
	 * @return true if a message is available
	 */
	virtual bool receiveMsgTimeout(can::CanMsg* pCMsg, int nMicroSecTimeout) = 0;

	/**
	 * Check if the current CAN interface was opened on OBJECT mode.
	 * @return true if opened in OBJECT mode, false if not.
	 */
	virtual bool isObjectMode() = 0;

	/**
	 * Set the CAN interface type. This is necessary to implement
	 * a proper CAN bus simulation.
	 * @param iType The CAN interface type.
	 */
	void setCanItfType(CanItfType iType) { m_iCanItfType = iType; }

	/**
	 * Get the CAN interface type. This is necessary to implement
	 * a proper CAN bus simulation.
	 * @return The CAN interface type.
	 */
	CanItfType getCanItfType() { return m_iCanItfType; }

private:
	/// The CAN interface type.
	CanItfType m_iCanItfType;
};
//-----------------------------------------------

#endif
