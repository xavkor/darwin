/*
 *   CM730.cpp
 *
 *   Author: ROBOTIS
 *
 */
#include <stdio.h>
#include "FSR.h"
#include "CM730.h"
#include "MotionStatus.h"
#include <stdlib.h>

#ifdef CN730_EMULATED
#include <sys/time.h>
#endif

using namespace Robot;


#define ID					(2)
#define LENGTH				(3)
#define INSTRUCTION			(4)
#define ERRBIT				(4)
#define PARAMETER			(5)
#define DEFAULT_BAUDNUMBER	(1)

#define INST_PING			(1)
#define INST_READ			(2)
#define INST_WRITE			(3)
#define INST_REG_WRITE		(4)
#define INST_ACTION			(5)
#define INST_RESET			(6)
#define INST_SYNC_WRITE		(131)   // 0x83
#define INST_BULK_READ      (146)   // 0x92


BulkReadData::BulkReadData() :
        start_address(0),
        length(0),
        error(-1)
{
    for(int i = 0; i < MX28::MAXNUM_ADDRESS; i++)
        table[i] = 0;
}

int BulkReadData::ReadByte(int address)
{
    if(address >= start_address && address < (start_address + length))
        return (int)table[address];

    return 0;
}

int BulkReadData::ReadWord(int address)
{
    if(address >= start_address && address < (start_address + length))
        return CM730::MakeWord(table[address], table[address+1]);

    return 0;
}


CM730::CM730(PlatformCM730 *platform)
{
	m_Platform = platform;
	DEBUG_PRINT = false;
	m_DelayedWords = 0;
	m_bIncludeTempData = false;
	m_BulkReadTxPacket[LENGTH] = 0;
	for(int i = 0; i < ID_BROADCAST; i++)
	    m_BulkReadData[i] = BulkReadData();

#ifdef CN730_EMULATED
	// Emulation support
	EMULATE_CM730 =  true;
	EMULATE_FSR =    true;
	EMULATE_MX28 =   false;
	EMULATE_AX12 =   false;
	EMULATE_RECORD = false;

	if(EMULATE_CM730 == true) 
	{
		emu_CM730_memory = new unsigned char[CM730::MAXNUM_ADDRESS];
		Reset_CM730_Emulation();
	}

	if(EMULATE_FSR == true)
	{
		emu_R_FSR_memory = new unsigned char[FSR::MAXNUM_ADDRESS];
		emu_L_FSR_memory = new unsigned char[FSR::MAXNUM_ADDRESS];
		Reset_FSR_Emulation();
	}

	if(EMULATE_MX28 == true) 
	{
		emu_MX28_memory = new unsigned char[EMULATED_SERVO_COUNT][EMULATED_SERVO_SIZE];
		Reset_MX28_Emulation();
	}

	if(EMULATE_AX12 == true) 
	{
		emu_AX12_memory = new unsigned char[EMULATED_SERVO_COUNT][EMULATED_SERVO_SIZE];
		Reset_AX12_Emulation();
	}
	// End emulation support
#endif
}

CM730::~CM730()
{
	Disconnect();

#ifdef CN730_EMULATED
	// Emulation support
	if(EMULATE_CM730 == true) delete emu_CM730_memory;

	if(EMULATE_FSR == true) {
		delete emu_R_FSR_memory;
		delete emu_L_FSR_memory;
	}

	if(EMULATE_MX28 == true) {
		delete emu_MX28_memory;
		if(logfileMX28 != 0) fclose(logfileMX28);
	}

	if(EMULATE_AX12 == true) {
		delete emu_AX12_memory;
		if(logfileAX12 != 0) fclose(logfileAX12);
	}
	// End emulation support
#endif

	exit(0);
}

int CM730::TxRxPacket(unsigned char *txpacket, unsigned char *rxpacket, int priority)
{
	if(priority > 1)
		m_Platform->LowPriorityWait();
	if(priority > 0)
		m_Platform->MidPriorityWait();
	m_Platform->HighPriorityWait();

	int res = TX_FAIL;
	int length = txpacket[LENGTH] + 4;

	txpacket[0] = 0xFF;
    txpacket[1] = 0xFF;
	txpacket[length - 1] = CalculateChecksum(txpacket);

	if(DEBUG_PRINT == true)
	{
		fprintf(stderr, "\nTX: ");
		for(int n=0; n<length; n++)
			fprintf(stderr, "%.2X ", txpacket[n]);

		fprintf(stderr, "INST: ");
		switch(txpacket[INSTRUCTION])
		{
		case INST_PING:
			fprintf(stderr, "PING\n");
			break;

		case INST_READ:
			fprintf(stderr, "READ\n");
			break;

		case INST_WRITE:
			fprintf(stderr, "WRITE\n");
			break;

		case INST_REG_WRITE:
			fprintf(stderr, "REG_WRITE\n");
			break;

		case INST_ACTION:
			fprintf(stderr, "ACTION\n");
			break;

		case INST_RESET:
			fprintf(stderr, "RESET\n");
			break;

		case INST_SYNC_WRITE:
			fprintf(stderr, "SYNC_WRITE\n");
			break;

        case INST_BULK_READ:
            fprintf(stderr, "BULK_READ\n");
            break;

		default:
			fprintf(stderr, "UNKNOWN\n");
			break;
		}
	}

	if(length < (MAXNUM_TXPARAM + 6))
	{
		m_Platform->ClearPort();
		if(m_Platform->WritePort(txpacket, length) == length)
		{
			if (txpacket[ID] != ID_BROADCAST)
			{
				int to_length = 0;

				if(txpacket[INSTRUCTION] == INST_READ)
					to_length = txpacket[PARAMETER+1] + 6;
				else
					to_length = 6;

				m_Platform->SetPacketTimeout(length);

				int get_length = 0;
				if(DEBUG_PRINT == true)
					fprintf(stderr, "RX: ");
				
				while(1)
				{
					length = m_Platform->ReadPort(&rxpacket[get_length], to_length - get_length);
					if(DEBUG_PRINT == true)
					{
						for(int n=0; n<length; n++)
							fprintf(stderr, "%.2X ", rxpacket[get_length + n]);
					}
					get_length += length;

					if(get_length == to_length)
					{
						// Find packet header
						int i;
						for(i = 0; i < (get_length - 1); i++)
						{
							if(rxpacket[i] == 0xFF && rxpacket[i+1] == 0xFF)
								break;
							else if(i == (get_length - 2) && rxpacket[get_length - 1] == 0xFF)
								break;
						}

						if(i == 0)
						{
							// Check checksum
							unsigned char checksum = CalculateChecksum(rxpacket);
							if(DEBUG_PRINT == true)
								fprintf(stderr, "CHK:%.2X\n", checksum);

							if(rxpacket[get_length-1] == checksum)
								res = SUCCESS;
							else
								res = RX_CORRUPT;
							
							break;
						}
						else
						{
							for(int j = 0; j < (get_length - i); j++)
								rxpacket[j] = rxpacket[j+i];
							get_length -= i;
						}						
					}
					else
					{
						if(m_Platform->IsPacketTimeout() == true)
						{
							if(get_length == 0)
								res = RX_TIMEOUT;
							else
								res = RX_CORRUPT;
							
							break;
						}
					}
				}
			}
			else if(txpacket[INSTRUCTION] == INST_BULK_READ)
			{
                int to_length = 0;
                int num = (txpacket[LENGTH]-3) / 3;

                for(int x = 0; x < num; x++)
                {
                    int _id = txpacket[PARAMETER+(3*x)+2];
                    int _len = txpacket[PARAMETER+(3*x)+1];
                    int _addr = txpacket[PARAMETER+(3*x)+3];

                    to_length += _len + 6;
                    m_BulkReadData[_id].length = _len;
                    m_BulkReadData[_id].start_address = _addr;
                }

                m_Platform->SetPacketTimeout(to_length*1.5);

                int get_length = 0;
                if(DEBUG_PRINT == true)
                    fprintf(stderr, "RX: ");

                while(1)
                {
                    length = m_Platform->ReadPort(&rxpacket[get_length], to_length - get_length);
                    if(DEBUG_PRINT == true)
                    {
                        for(int n=0; n<length; n++)
                            fprintf(stderr, "%.2X ", rxpacket[get_length + n]);
                    }
                    get_length += length;

                    if(get_length == to_length)
                    {
                        res = SUCCESS;
                        break;
                    }
                    else
                    {
                        if(m_Platform->IsPacketTimeout() == true)
                        {
                            if(get_length == 0)
                                res = RX_TIMEOUT;
                            else
                                res = RX_CORRUPT;

                            break;
                        }
                    }
                }

                for(int x = 0; x < num; x++)
                {
                    int _id = txpacket[PARAMETER+(3*x)+2];
                    m_BulkReadData[_id].error = -1;
                }

                while(1)
                {
                    int i;
                    for(i = 0; i < get_length - 1; i++)
                    {
                        if(rxpacket[i] == 0xFF && rxpacket[i+1] == 0xFF)
                            break;
                        else if(i == (get_length - 2) && rxpacket[get_length - 1] == 0xFF)
                            break;
                    }

                    if(i == 0)
                    {
                        // Check checksum
                        unsigned char checksum = CalculateChecksum(rxpacket);
                        if(DEBUG_PRINT == true)
                            fprintf(stderr, "CHK:%.2X\n", checksum);

                        if(rxpacket[LENGTH+rxpacket[LENGTH]] == checksum)
                        {
                            for(int j = 0; j < (rxpacket[LENGTH]-2); j++)
                                m_BulkReadData[rxpacket[ID]].table[m_BulkReadData[rxpacket[ID]].start_address + j] = rxpacket[PARAMETER + j];

                            m_BulkReadData[rxpacket[ID]].error = (int)rxpacket[ERRBIT];

                            int cur_packet_length = LENGTH + 1 + rxpacket[LENGTH];
                            to_length = get_length - cur_packet_length;
                            for(int j = 0; j <= to_length; j++)
                                rxpacket[j] = rxpacket[j+cur_packet_length];

                            get_length = to_length;
                            num--;
                        }
                        else
                        {
                            res = RX_CORRUPT;

                            for(int j = 0; j <= get_length - 2; j++)
                                rxpacket[j] = rxpacket[j+2];

                            to_length = get_length -= 2;
                        }

                        if(num == 0)
                            break;
                        else if(get_length <= 6)
                        {
                            if(num != 0) res = RX_CORRUPT;
                            break;
                        }

                    }
                    else
                    {
                        for(int j = 0; j < (get_length - i); j++)
                            rxpacket[j] = rxpacket[j+i];
                        get_length -= i;
                    }
                }
			}
			else
				res = SUCCESS;			
		}
		else
			res = TX_FAIL;		
	}
	else
		res = TX_CORRUPT;

	if(DEBUG_PRINT == true)
	{
		fprintf(stderr, "Time:%.2fms  ", m_Platform->GetPacketTime());
		fprintf(stderr, "RETURN: ");
		switch(res)
		{
		case SUCCESS:
			fprintf(stderr, "SUCCESS\n");
			break;

		case TX_CORRUPT:
			fprintf(stderr, "TX_CORRUPT\n");
			break;

		case TX_FAIL:
			fprintf(stderr, "TX_FAIL\n");
			break;

		case RX_FAIL:
			fprintf(stderr, "RX_FAIL\n");
			break;

		case RX_TIMEOUT:
			fprintf(stderr, "RX_TIMEOUT\n");
			break;

		case RX_CORRUPT:
			fprintf(stderr, "RX_CORRUPT\n");
			break;

		default:
			fprintf(stderr, "UNKNOWN\n");
			break;
		}
	}

	m_Platform->HighPriorityRelease();
    if(priority > 0)
        m_Platform->MidPriorityRelease();
    if(priority > 1)
        m_Platform->LowPriorityRelease();

	return res;
}

unsigned char CM730::CalculateChecksum(unsigned char *packet)
{
	unsigned char checksum = 0x00;
	for(int i=2; i<packet[LENGTH]+3; i++ )
		checksum += packet[i];
	return (~checksum);
}

void CM730::MakeBulkReadPacket()
{
    int number = 0;

    m_BulkReadTxPacket[ID]              = (unsigned char)ID_BROADCAST;
    m_BulkReadTxPacket[INSTRUCTION]     = INST_BULK_READ;
    m_BulkReadTxPacket[PARAMETER]       = (unsigned char)0x0;

    //if(Ping(CM730::ID_CM, 0) == SUCCESS)
    {
        m_BulkReadTxPacket[PARAMETER+3*number+1] = 30;
        m_BulkReadTxPacket[PARAMETER+3*number+2] = CM730::ID_CM;
        m_BulkReadTxPacket[PARAMETER+3*number+3] = CM730::P_DXL_POWER;
        number++;
    }

//    for(int id = 1; id < JointData::NUMBER_OF_JOINTS; id++)
//    {
//        if(MotionStatus::m_CurrentJoints.GetEnable(id))
//        {
//            m_BulkReadTxPacket[PARAMETER+3*number+1] = 2;   // length
//            m_BulkReadTxPacket[PARAMETER+3*number+2] = id;  // id
//            m_BulkReadTxPacket[PARAMETER+3*number+3] = MX28::P_PRESENT_POSITION_L; // start address
//            number++;
//        }
//    }
    if(m_bIncludeTempData == true)
			{
			for(int id = JointData::ID_MIN; id <= JointData::ID_MAX; id++)
				{
				if(MotionStatus::m_CurrentJoints.GetEnable(id))
					{
					m_BulkReadTxPacket[PARAMETER+3*number+1] = 1;   // length
					m_BulkReadTxPacket[PARAMETER+3*number+2] = id;  // id
					m_BulkReadTxPacket[PARAMETER+3*number+3] = MX28::P_PRESENT_TEMPERATURE; // start address
					number++;
					}
				}
			}
		/*
    if(Ping(FSR::ID_L_FSR, 0) == SUCCESS)
    {
        m_BulkReadTxPacket[PARAMETER+3*number+1] = 2;               // length
        m_BulkReadTxPacket[PARAMETER+3*number+2] = FSR::ID_L_FSR;   // id
        m_BulkReadTxPacket[PARAMETER+3*number+3] = FSR::P_FSR_X;    // start address
        number++;
    }

    if(Ping(FSR::ID_R_FSR, 0) == SUCCESS)
    {
        m_BulkReadTxPacket[PARAMETER+3*number+1] = 2;               // length
        m_BulkReadTxPacket[PARAMETER+3*number+2] = FSR::ID_R_FSR;   // id
        m_BulkReadTxPacket[PARAMETER+3*number+3] = FSR::P_FSR_X;    // start address
        number++;
    }
*/
    //fprintf(stderr, "NUMBER : %d \n", number);

    m_BulkReadTxPacket[LENGTH]          = (number * 3) + 3;
}

int CM730::BulkRead()
{
    unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0, };

#ifdef CN730_EMULATED
	// Emulation support
	if(EMULATE_CM730 == true) {
 
		if(m_BulkReadTxPacket[LENGTH] == 0) {
			MakeBulkReadPacket();
        		return TX_FAIL;
    	}

		for(int i=0; i<ID_BROADCAST; i++) m_BulkReadData[i].error = -1;

		int num = (m_BulkReadTxPacket[LENGTH]-3) / 3;

        for(int i=0; i<num; i++)
        {
    		int len = m_BulkReadTxPacket[PARAMETER+(3*i)+1];
        	int id = m_BulkReadTxPacket[PARAMETER+(3*i)+2];
        	int addr = m_BulkReadTxPacket[PARAMETER+(3*i)+3];

    		if(id == CM730::ID_CM) {
                m_BulkReadData[id].length = len;
                m_BulkReadData[id].start_address = addr;
                m_BulkReadData[id].error = 0;
        		for(int j=0; j<len; j++) {
                    m_BulkReadData[id].table[j] = emu_CM730_memory[addr+j];
         		}
        	}

    		if((EMULATE_FSR == true) && (id == FSR::ID_R_FSR)) {
                m_BulkReadData[id].length = len;
                m_BulkReadData[id].start_address = addr;
                m_BulkReadData[id].error = 0;
        		for(int j=0; j<len; j++) {
                    m_BulkReadData[id].table[j] = emu_R_FSR_memory[addr+j];
         		}
        	}

    		if((EMULATE_FSR == true) && (id == FSR::ID_L_FSR)) {
                m_BulkReadData[id].length = len;
                m_BulkReadData[id].start_address = addr;
                m_BulkReadData[id].error = 0;
        		for(int j=0; j<len; j++) {
                    m_BulkReadData[id].table[j] = emu_L_FSR_memory[addr+j];
         		}
        	}

     		if((EMULATE_MX28 == true) && (id < EMULATED_SERVO_COUNT) && MX28::isMX28(id)) {
				m_BulkReadData[id].length = len;
				m_BulkReadData[id].start_address = addr;
				m_BulkReadData[id].error = 0;
				for(int j=0; j<len; j++) {
					m_BulkReadData[id].table[j] = emu_MX28_memory[id][addr+j];
				}
    		}

     		if((EMULATE_AX12 == true) && (id < EMULATED_SERVO_COUNT) && AX12::isAX12(id)) {
				m_BulkReadData[id].length = len;
				m_BulkReadData[id].start_address = addr;
				m_BulkReadData[id].error = 0;
				for(int j=0; j<len; j++) {
					m_BulkReadData[id].table[j] = emu_AX12_memory[id][addr+j];
				}
    		}

        }

		return SUCCESS;

	}
	// End emulation support
#endif

    if(m_BulkReadTxPacket[LENGTH] != 0)
        return TxRxPacket(m_BulkReadTxPacket, rxpacket, 0);
    else
    {
				MakeBulkReadPacket();
        return TX_FAIL;
    }
}

int CM730::SyncWrite(int start_addr, int each_length, int number, int *pParam)
{
	unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0, };
	unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0, };
	int n;

#ifdef CN730_EMULATED
	// Emulation support
	if(EMULATE_MX28 == true) 
	{
		for(int i=0; i<number; i++) {
			int index = i * each_length;
			int id = (unsigned char) pParam[index];
			if(id < EMULATED_SERVO_COUNT && MX28::isMX28(id)) 
			{
				for(int j=0; j<each_length-1; j++) {
					emu_MX28_memory[id][start_addr+j] = (unsigned char)pParam[index+j+1];
					if((start_addr+j) == MX28::P_GOAL_POSITION_L) {
						emu_MX28_memory[id][MX28::P_PRESENT_POSITION_L] = (unsigned char)pParam[index+j+1];
						emu_MX28_memory[id][MX28::P_PRESENT_POSITION_H] = (unsigned char)pParam[index+j+2];
					}
				}
			}
		}

		if((EMULATE_RECORD == true) && (logfileMX28 != 0)) {
			itimerval * time_left = NULL;
			setitimer(ITIMER_REAL, 0, time_left);
			fprintf(logfileMX28, "%f", simTime);
			simTime += simIncrement;
			for(int i=0; i<EMULATED_SERVO_COUNT; i++) {
            if(MX28::isMX28(i)){
				   int d = emu_MX28_memory[i][MX28::P_GOAL_POSITION_H] << 8;
				   d += emu_MX28_memory[i][MX28::P_GOAL_POSITION_L] & 0xFF;
				   fprintf(logfileMX28, " %d", d);
            }
			}
			putc('\n', logfileMX28);
			setitimer(ITIMER_REAL, time_left, 0);
		}

		return SUCCESS;
	}

	if(EMULATE_AX12 == true) 
	{
		for(int i=0; i<number; i++) {
			int index = i * each_length;
			int id = (unsigned char) pParam[index];
			if(id < EMULATED_SERVO_COUNT && AX12::isAX12(id)) 
			{
				for(int j=0; j<each_length-1; j++) {
					emu_AX12_memory[id][start_addr+j] = (unsigned char)pParam[index+j+1];
					if((start_addr+j) == AX12::P_GOAL_POSITION_L) {
						emu_AX12_memory[id][AX12::P_PRESENT_POSITION_L] = (unsigned char)pParam[index+j+1];
						emu_AX12_memory[id][AX12::P_PRESENT_POSITION_H] = (unsigned char)pParam[index+j+2];
					}
				}
			}
		}

		if((EMULATE_RECORD == true) && (logfileAX12 != 0)) {
			itimerval * time_left = NULL;
			setitimer(ITIMER_REAL, 0, time_left);
			fprintf(logfileAX12, "%f", simTime);
			simTime += simIncrement;
			for(int i=0; i<EMULATED_SERVO_COUNT; i++) {
            if(AX12::isAX12(i)){
				   int d = emu_AX12_memory[i][AX12::P_GOAL_POSITION_H] << 8;
				   d += emu_AX12_memory[i][AX12::P_GOAL_POSITION_L] & 0xFF;
				   fprintf(logfileAX12, " %d", d);
            }
			}
			putc('\n', logfileAX12);
			setitimer(ITIMER_REAL, time_left, 0);
		}

		return SUCCESS;
	}
	// End emulation support
#endif

    txpacket[ID]                = (unsigned char)ID_BROADCAST;
    txpacket[INSTRUCTION]       = INST_SYNC_WRITE;
    txpacket[PARAMETER]			= (unsigned char)start_addr;
    txpacket[PARAMETER + 1]		= (unsigned char)(each_length - 1);
    for(n = 0; n < (number * each_length); n++)
        txpacket[PARAMETER + 2 + n]   = (unsigned char)pParam[n];
    txpacket[LENGTH]            = n + 4;

    return TxRxPacket(txpacket, rxpacket, 0);
}

bool CM730::Connect()
{
#ifdef CN730_EMULATED
	// Emulation support
	if(EMULATE_MX28 == true) return DXLPowerOn();
	if(EMULATE_AX12 == true) return DXLPowerOn();
	// End emulation support
#endif

	if(m_Platform->OpenPort() == false)
	{
        fprintf(stderr, "\n Fail to open port\n");
        fprintf(stderr, " CM-730 is used by another program or do not have root privileges.\n\n");
		return false;
	}

	return DXLPowerOn();
}

bool CM730::DXLPowerOn(bool state)
{
	if(WriteByte(CM730::ID_CM, CM730::P_DXL_POWER, state==true?1:0, 0) == CM730::SUCCESS)
	{
		if(DEBUG_PRINT == true)
			fprintf(stderr, " Succeed to change Dynamixel power!\n");
		
		WriteWord(CM730::ID_CM, CM730::P_LED_HEAD_L, state==true?MakeColor(1, 1, 1):MakeColor(0, 0, 0), 0);
		m_Platform->Sleep(300); // about 300msec
	}
	else
	{
		if(DEBUG_PRINT == true)
			fprintf(stderr, " Fail to change Dynamixel power!\n");
		return false;
	}

	return true;
}

void CM730::Disconnect()
{
    // Make the Head LED to green
	//WriteWord(CM730::ID_CM, CM730::P_LED_HEAD_L, MakeColor(0, 255, 0), 0);
	unsigned char txpacket[] = {0xFF, 0xFF, 0xC8, 0x05, 0x03, 0x1A, 0xE0, 0x03, 0x32};
	m_Platform->WritePort(txpacket, 9);

#ifdef CN730_EMULATED
	// Emulation support
	if(EMULATE_MX28 == true) return;
	if(EMULATE_AX12 == true) return;
	// End emulation support
#endif

	m_Platform->ClosePort();
}

int CM730::WriteByte(int address, int value, int *error)
{
	return WriteByte(ID_CM, address, value, error);
}

int CM730::WriteWord(int address, int value, int *error)
{
	return WriteWord(ID_CM, address, value, error);
}

void CM730::WriteWordDelayed(int address,int value)
{
	if(m_DelayedWords > 9) return;

	m_DelayedWord[m_DelayedWords] = value;
	m_DelayedAddress[m_DelayedWords] = address;
	m_DelayedWords++;
	return;
}

int CM730::Ping(int id, int *error)
{
	unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0, };
	unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0, };
	int result;

#ifdef CN730_EMULATED
	// Emulation support
	if((EMULATE_CM730 == true) && (id == emu_CM730_memory[P_ID])) return SUCCESS;

	if((EMULATE_FSR == true) && (id == FSR::ID_R_FSR)) return SUCCESS;
	if((EMULATE_FSR == true) && (id == FSR::ID_L_FSR)) return SUCCESS;

   if(MX28::isMX28(id))
      if((EMULATE_MX28 == true) && (id < EMULATED_SERVO_COUNT)) return SUCCESS;
   if(AX12::isAX12(id))
	   if((EMULATE_AX12 == true) && (id < EMULATED_SERVO_COUNT)) return SUCCESS;
	// End emulation support
#endif

    txpacket[ID]           = (unsigned char)id;
    txpacket[INSTRUCTION]  = INST_PING;
    txpacket[LENGTH]       = 2;

	result = TxRxPacket(txpacket, rxpacket, 2);
	if(result == SUCCESS && txpacket[ID] != ID_BROADCAST)
	{		
		if(error != 0)
			*error = (int)rxpacket[ERRBIT];
	}

	return result;
}

int CM730::ReadByte(int id, int address, int *pValue, int *error)
{
	unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0, };
	unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0, };
	int result;

#ifdef CN730_EMULATED
	// Emulation support
	if(EMULATE_CM730 == true) 
	{
		if(id == emu_CM730_memory[P_ID])
		{
			 *pValue = (int) emu_CM730_memory[address];
			return SUCCESS;
		}
	}

	if(EMULATE_FSR == true)
	{
		if(id == FSR::FSR::ID_R_FSR)
		{
			 *pValue = (int) emu_R_FSR_memory[address];
			return SUCCESS;
		}
		if(id == FSR::FSR::ID_L_FSR)
		{
			 *pValue = (int) emu_L_FSR_memory[address];
			return SUCCESS;
		}
	}

	if(EMULATE_MX28 == true) 
	{
		if(id < EMULATED_SERVO_COUNT && MX28::isMX28(id)) 
		{
			 *pValue = (int) emu_MX28_memory[id][address];			
			return SUCCESS;
		}
	} 

	if(EMULATE_AX12 == true) 
	{
		if(id < EMULATED_SERVO_COUNT && AX12::isAX12(id)) 
		{
			 *pValue = (int) emu_AX12_memory[id][address];			
			return SUCCESS;
		}
	} 
	// End emulation support
#endif

    txpacket[ID]           = (unsigned char)id;
    txpacket[INSTRUCTION]  = INST_READ;
	txpacket[PARAMETER]    = (unsigned char)address;
    txpacket[PARAMETER+1]  = 1;
    txpacket[LENGTH]       = 4;

	result = TxRxPacket(txpacket, rxpacket, 2);
	if(result == SUCCESS)
	{
		*pValue = (int)rxpacket[PARAMETER];
		if(error != 0)
			*error = (int)rxpacket[ERRBIT];
	}

	return result;
}

int CM730::ReadWord(int id, int address, int *pValue, int *error)
{
	unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0, };
	unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0, };
	int result;

#ifdef CN730_EMULATED
	// Emulation support
	if(EMULATE_CM730 == true) 
	{
		if(id == emu_CM730_memory[P_ID])
		{
			 *pValue = MakeWord((int) emu_CM730_memory[address], (int) emu_CM730_memory[address+1]);
			return SUCCESS;
		}
	}

	if(EMULATE_FSR == true)
	{
		if(id == FSR::FSR::ID_R_FSR)
		{
			 *pValue = MakeWord((int) emu_R_FSR_memory[address], (int) emu_R_FSR_memory[address+1]);
			return SUCCESS;
		}
		if(id == FSR::FSR::ID_L_FSR)
		{
			 *pValue = MakeWord((int) emu_L_FSR_memory[address], (int) emu_L_FSR_memory[address+1]);
			return SUCCESS;
		}
	}

	if(EMULATE_MX28 == true) 
	{
		if(id < EMULATED_SERVO_COUNT && MX28::isMX28(id))
		{
			*pValue = MakeWord((int) emu_MX28_memory[id][address], (int) emu_MX28_memory[id][address+1]);
			return SUCCESS;
		}
	} 

	if(EMULATE_AX12 == true) 
	{
		if(id < EMULATED_SERVO_COUNT && AX12::isAX12(id))
		{
			*pValue = MakeWord((int) emu_AX12_memory[id][address], (int) emu_AX12_memory[id][address+1]);
			return SUCCESS;
		}
	} 
	// End emulation support
#endif

    txpacket[ID]           = (unsigned char)id;
    txpacket[INSTRUCTION]  = INST_READ;
	txpacket[PARAMETER]    = (unsigned char)address;
    txpacket[PARAMETER+1]  = 2;
    txpacket[LENGTH]       = 4;

	result = TxRxPacket(txpacket, rxpacket, 2);
	if(result == SUCCESS)
	{
		*pValue = MakeWord((int)rxpacket[PARAMETER], (int)rxpacket[PARAMETER + 1]);

		if(error != 0)
			*error = (int)rxpacket[ERRBIT];
	}

	return result;
}

int CM730::ReadTable(int id, int start_addr, int end_addr, unsigned char *table, int *error)
{
	unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0, };
	unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0, };
	int result;
	int length = end_addr - start_addr + 1;

#ifdef CN730_EMULATED
	// Emulation support
	if(EMULATE_CM730 == true) 
	{
		if(id == emu_CM730_memory[P_ID])
		{
			for (int i=start_addr; i <= end_addr; i++) table[i] = emu_CM730_memory[i];
			return SUCCESS;
		}
	}

	if(EMULATE_FSR == true)
	{
		if(id == FSR::FSR::ID_R_FSR)
		{
			for (int i=start_addr; i <= end_addr; i++) table[i] = emu_R_FSR_memory[i];
			return SUCCESS;
		}
		if(id == FSR::FSR::ID_L_FSR)
		{
			for (int i=start_addr; i <= end_addr; i++) table[i] = emu_L_FSR_memory[i];
			return SUCCESS;
		}
	}

	if(EMULATE_MX28 == true) 
	{
		if(id < EMULATED_SERVO_COUNT && MX28::isMX28(id))
		{
			for (int i=start_addr; i <= end_addr; i++) table[i] = emu_MX28_memory[id][i];			
			return SUCCESS;
		}
	} 

	if(EMULATE_AX12 == true) 
	{
		if(id < EMULATED_SERVO_COUNT && AX12::isAX12(id))
		{
			for (int i=start_addr; i <= end_addr; i++) table[i] = emu_AX12_memory[id][i];			
			return SUCCESS;
		}
	} 
	// End emulation support
#endif
    txpacket[ID]           = (unsigned char)id;
    txpacket[INSTRUCTION]  = INST_READ;
	txpacket[PARAMETER]    = (unsigned char)start_addr;
    txpacket[PARAMETER+1]  = (unsigned char)length;
    txpacket[LENGTH]       = 4;

	result = TxRxPacket(txpacket, rxpacket, 1);
	if(result == SUCCESS)
	{
		for(int i=0; i<length; i++)
			table[start_addr + i] = rxpacket[PARAMETER + i];

		if(error != 0)
			*error = (int)rxpacket[ERRBIT];
	}

	return result;
}

int CM730::WriteByte(int id, int address, int value, int *error)
{
	unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0, };
	unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0, };
	int result;

#ifdef CN730_EMULATED
	// Emulation support
	if(EMULATE_CM730 == true) 
	{
		if(id == emu_CM730_memory[P_ID])
		{
			emu_CM730_memory[address] = value;
			return SUCCESS;
		}
	}

	if(EMULATE_FSR == true)
	{
		if(id == FSR::FSR::ID_R_FSR)
		{
			emu_R_FSR_memory[address] = value;
			return SUCCESS;
		}
		if(id == FSR::FSR::ID_L_FSR)
		{
			emu_L_FSR_memory[address] = value;
			return SUCCESS;
		}
	}

	if(EMULATE_MX28 == true) 
	{
		if(id < EMULATED_SERVO_COUNT && MX28::isMX28(id))
		{
			emu_MX28_memory[id][address] = value;
			return SUCCESS;
		}
	}

	if(EMULATE_AX12 == true) 
	{
		if(id < EMULATED_SERVO_COUNT && AX12::isAX12(id))
		{
			emu_AX12_memory[id][address] = value;
			return SUCCESS;
		}
	}
	// End emulation support
#endif

    txpacket[ID]           = (unsigned char)id;
    txpacket[INSTRUCTION]  = INST_WRITE;
	txpacket[PARAMETER]    = (unsigned char)address;
    txpacket[PARAMETER+1]  = (unsigned char)value;
    txpacket[LENGTH]       = 4;

	result = TxRxPacket(txpacket, rxpacket, 2);
	if(result == SUCCESS && id != ID_BROADCAST)
	{
		if(error != 0)
			*error = (int)rxpacket[ERRBIT];
	}

	return result;
}

int CM730::WriteWord(int id, int address, int value, int *error)
{
	unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0, };
	unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0, };
	int result;

#ifdef CN730_EMULATED
	// Emulation support
	if(EMULATE_CM730 == true)
	{
		if(id == emu_CM730_memory[P_ID])
		{
			emu_CM730_memory[address] = (unsigned char)GetLowByte(value);
			emu_CM730_memory[address +1] = (unsigned char)GetHighByte(value);
			return SUCCESS;
		}
	}

	if(EMULATE_FSR == true)
	{
		if(id == FSR::FSR::ID_R_FSR)
		{
			emu_R_FSR_memory[address] = (unsigned char)GetLowByte(value);
			emu_R_FSR_memory[address +1] = (unsigned char)GetHighByte(value);
			return SUCCESS;
		}
		if(id == FSR::FSR::ID_L_FSR)
		{
			emu_L_FSR_memory[address] = (unsigned char)GetLowByte(value);
			emu_L_FSR_memory[address +1] = (unsigned char)GetHighByte(value);
			return SUCCESS;
		}
	}

	if(EMULATE_MX28 == true) 
	{
		if(id == CM730::ID_BROADCAST) 
		{
			for(int i=0; i<EMULATED_SERVO_COUNT; i++) {
            if(MX28::isMX28(id)){
				   emu_MX28_memory[i][address] = (unsigned char)GetLowByte(value);
				   emu_MX28_memory[i][address+1] = (unsigned char)GetHighByte(value);
				   if(address == MX28::P_GOAL_POSITION_L) {
					   emu_MX28_memory[i][MX28::P_PRESENT_POSITION_L] = (unsigned char)GetLowByte(value);
					   emu_MX28_memory[i][MX28::P_PRESENT_POSITION_H] = (unsigned char)GetHighByte(value);
				   }
            }
			}
			return SUCCESS;
		}
		if(id < EMULATED_SERVO_COUNT && MX28::isMX28(id))
		{
			emu_MX28_memory[id][address] = (unsigned char)GetLowByte(value);
			emu_MX28_memory[id][address+1] = (unsigned char)GetHighByte(value);
			if(address == MX28::P_GOAL_POSITION_L) {
				emu_MX28_memory[id][MX28::P_PRESENT_POSITION_L] = (unsigned char)GetLowByte(value);
				emu_MX28_memory[id][MX28::P_PRESENT_POSITION_H] = (unsigned char)GetHighByte(value);
			}
			return SUCCESS;
		}
	}

	if(EMULATE_AX12 == true) 
	{
		if(id == CM730::ID_BROADCAST) 
		{
			for(int i=0; i<EMULATED_SERVO_COUNT; i++) {
            if(AX12::isAX12(id)){
				   emu_AX12_memory[i][address] = (unsigned char)GetLowByte(value);
				   emu_AX12_memory[i][address+1] = (unsigned char)GetHighByte(value);
				   if(address == AX12::P_GOAL_POSITION_L) {
					   emu_AX12_memory[i][AX12::P_PRESENT_POSITION_L] = (unsigned char)GetLowByte(value);
					   emu_AX12_memory[i][AX12::P_PRESENT_POSITION_H] = (unsigned char)GetHighByte(value);
				   }
            }
			}
			return SUCCESS;
		}
		if(id < EMULATED_SERVO_COUNT && AX12::isAX12(id)) 
		{
			emu_AX12_memory[id][address] = (unsigned char)GetLowByte(value);
			emu_AX12_memory[id][address+1] = (unsigned char)GetHighByte(value);
			if(address == AX12::P_GOAL_POSITION_L) {
				emu_AX12_memory[id][AX12::P_PRESENT_POSITION_L] = (unsigned char)GetLowByte(value);
				emu_AX12_memory[id][AX12::P_PRESENT_POSITION_H] = (unsigned char)GetHighByte(value);
			}
			return SUCCESS;
		}
	}
	// End emulation support
#endif

    txpacket[ID]           = (unsigned char)id;
    txpacket[INSTRUCTION]  = INST_WRITE;
	txpacket[PARAMETER]    = (unsigned char)address;
    txpacket[PARAMETER+1]  = (unsigned char)GetLowByte(value);
	txpacket[PARAMETER+2]  = (unsigned char)GetHighByte(value);
    txpacket[LENGTH]       = 5;

	result = TxRxPacket(txpacket, rxpacket, 2);
	if(result == SUCCESS && id != ID_BROADCAST)
	{
		if(error != 0)
			*error = (int)rxpacket[ERRBIT];
	}

	return result;
}

int CM730::MakeWord(int lowbyte, int highbyte)
{
	unsigned short word;

    word = highbyte;
    word = word << 8;
    word = word + lowbyte;

    return (int)word;
}

int CM730::GetLowByte(int word)
{
	unsigned short temp;
    temp = word & 0xff;
    return (int)temp;
}

int CM730::GetHighByte(int word)
{
	unsigned short temp;
    temp = word & 0xff00;
    return (int)(temp >> 8);
}

// 5 bits per color
int CM730::MakeColor(int red, int green, int blue)
{
	int r = red & 0x1F;
	int g = green & 0x1F;
	int b = blue & 0x1F;

	return (int)(r|(g<<5)|(b<<10));
}

#ifdef CN730_EMULATED
// Emulation support
void CM730::Reset_CM730_Emulation(void) {

	for(int i=0; i<CM730::MAXNUM_ADDRESS; i++) emu_CM730_memory[i] = 0;
	
	emu_CM730_memory[CM730::P_MODEL_NUMBER_L] =     0x00;
	emu_CM730_memory[CM730::P_MODEL_NUMBER_H] =     0x73;
	emu_CM730_memory[CM730::P_ID] =                 0xC8;
	emu_CM730_memory[CM730::P_BAUD_RATE] =          0x01;
	emu_CM730_memory[CM730::P_RETURN_DELAY_TIME] =  0x00;
	emu_CM730_memory[CM730::P_RETURN_LEVEL] =       0x02;
	emu_CM730_memory[CM730::P_GYRO_Z_L] =           0x00;
	emu_CM730_memory[CM730::P_GYRO_Z_H] =           0x02;
	emu_CM730_memory[CM730::P_GYRO_Y_L] =           0x00;
	emu_CM730_memory[CM730::P_GYRO_Y_H] =           0x02;
	emu_CM730_memory[CM730::P_GYRO_X_L] =           0x00;
	emu_CM730_memory[CM730::P_GYRO_X_H] =           0x02;
	emu_CM730_memory[CM730::P_ACCEL_Z_L] =          0x00;
	emu_CM730_memory[CM730::P_ACCEL_Z_H] =          0x02;
	emu_CM730_memory[CM730::P_ACCEL_Y_L] =          0x00;
	emu_CM730_memory[CM730::P_ACCEL_Y_H] =          0x02;
	emu_CM730_memory[CM730::P_ACCEL_X_L] =          0x00;
	emu_CM730_memory[CM730::P_ACCEL_X_H] =          0x02;

	return;
}

void CM730::Reset_FSR_Emulation(void) {

	for(int i=0; i<FSR::MAXNUM_ADDRESS; i++) {
		emu_R_FSR_memory[i] = 0;
	}

	emu_R_FSR_memory[FSR::P_MODEL_NUMBER_L] = 0x00;
	emu_R_FSR_memory[FSR::P_MODEL_NUMBER_H] = 0x0C;
	emu_R_FSR_memory[FSR::P_ID] =             0xC8;
	emu_R_FSR_memory[FSR::P_BAUD_RATE] =      0x01;
	emu_R_FSR_memory[FSR::P_RETURN_LEVEL] =   0x02;

	for(int i=0; i<FSR::MAXNUM_ADDRESS; i++) {
		emu_L_FSR_memory[i] = 0;
	}

	emu_L_FSR_memory[FSR::P_MODEL_NUMBER_L] = 0x00;
	emu_L_FSR_memory[FSR::P_MODEL_NUMBER_H] = 0x0C;
	emu_L_FSR_memory[FSR::P_ID] =             0xC8;
	emu_L_FSR_memory[FSR::P_BAUD_RATE] =      0x01;
	emu_L_FSR_memory[FSR::P_RETURN_LEVEL] =   0x02;

	return;
}

void CM730::Reset_MX28_Emulation(void) {

	for(int i=0; i<EMULATED_SERVO_COUNT; i++)
	{
		if(MX28::isMX28(i)){
		   for(int j=0; j<MX28::MAXNUM_ADDRESS; j++) emu_MX28_memory[i][j] = 0;
		
		   emu_MX28_memory[i][MX28::P_MODEL_NUMBER_L] =          0x00;			
		   emu_MX28_memory[i][MX28::P_MODEL_NUMBER_H] =          0x0C;			
		   emu_MX28_memory[i][MX28::P_VERSION] =                 0x00;			
		   emu_MX28_memory[i][MX28::P_ID] =                      i;			
		   emu_MX28_memory[i][MX28::P_BAUD_RATE] =               0x01;			
		   emu_MX28_memory[i][MX28::P_RETURN_DELAY_TIME] =       0x00;			
		   emu_MX28_memory[i][MX28::P_CW_ANGLE_LIMIT_L] =        0x00;			
		   emu_MX28_memory[i][MX28::P_CW_ANGLE_LIMIT_H] =        0x00;			
		   emu_MX28_memory[i][MX28::P_CCW_ANGLE_LIMIT_L] =       0xFF;			
		   emu_MX28_memory[i][MX28::P_CCW_ANGLE_LIMIT_H] =       0x03;			
		   emu_MX28_memory[i][MX28::P_HIGH_LIMIT_TEMPERATURE] =  0x55;			
		   emu_MX28_memory[i][MX28::P_LOW_LIMIT_VOLTAGE] =       0x3C;			
		   emu_MX28_memory[i][MX28::P_HIGH_LIMIT_VOLTAGE] =      0xBE;			
		   emu_MX28_memory[i][MX28::P_MAX_TORQUE_L] =            0xFF;			
		   emu_MX28_memory[i][MX28::P_MAX_TORQUE_H] =            0x03;			
		   emu_MX28_memory[i][MX28::P_RETURN_LEVEL] =            0x02;			
		   emu_MX28_memory[i][MX28::P_ALARM_LED] =               0x04;					
		   emu_MX28_memory[i][MX28::P_PUNCH_L] =                 0x20;			
		   emu_MX28_memory[i][MX28::P_PUNCH_H] =                 0x00;		
      }	
	}

	EMULATE_RECORD = false;

	if(logfileMX28 != 0) fclose(logfileMX28);

	logfileMX28 = fopen("simulation_MX28.pos", "w");

	if((logfileMX28 == 0) && (DEBUG_PRINT == true)) fprintf(stderr, "Can not create Simulation file!\n");

	simTime = 0.0;
	simIncrement = 0.008;

	return;

}

void CM730::Reset_AX12_Emulation(void) {

	for(int i=0; i<EMULATED_SERVO_COUNT; i++)
	{
		if(AX12::isAX12(i)) {
		   for(int j=0; j<AX12::MAXNUM_ADDRESS; j++) emu_AX12_memory[i][j] = 0;
		
		   emu_AX12_memory[i][AX12::P_MODEL_NUMBER_L] =          0x00;			
		   emu_AX12_memory[i][AX12::P_MODEL_NUMBER_H] =          0x0C;			
		   emu_AX12_memory[i][AX12::P_VERSION] =                 0x00;			
		   emu_AX12_memory[i][AX12::P_ID] =                      i;			
		   emu_AX12_memory[i][AX12::P_BAUD_RATE] =               0x01;			
		   emu_AX12_memory[i][AX12::P_RETURN_DELAY_TIME] =       0x00;			
		   emu_AX12_memory[i][AX12::P_CW_ANGLE_LIMIT_L] =        0x00;			
		   emu_AX12_memory[i][AX12::P_CW_ANGLE_LIMIT_H] =        0x00;			
		   emu_AX12_memory[i][AX12::P_CCW_ANGLE_LIMIT_L] =       0xFF;			
		   emu_AX12_memory[i][AX12::P_CCW_ANGLE_LIMIT_H] =       0x03;			
		   emu_AX12_memory[i][AX12::P_HIGH_LIMIT_TEMPERATURE] =  0x55;			
		   emu_AX12_memory[i][AX12::P_LOW_LIMIT_VOLTAGE] =       0x3C;			
		   emu_AX12_memory[i][AX12::P_HIGH_LIMIT_VOLTAGE] =      0xBE;			
		   emu_AX12_memory[i][AX12::P_MAX_TORQUE_L] =            0xFF;			
		   emu_AX12_memory[i][AX12::P_MAX_TORQUE_H] =            0x03;			
		   emu_AX12_memory[i][AX12::P_RETURN_LEVEL] =            0x02;			
		   emu_AX12_memory[i][AX12::P_ALARM_LED] =               0x04;			
		   emu_AX12_memory[i][AX12::P_CW_COMPLIANCE_MARGIN] =    0x01;			
		   emu_AX12_memory[i][AX12::P_CCW_COMPLIANCE_MARGIN] =   0x01;			
		   emu_AX12_memory[i][AX12::P_CW_COMPLIANCE_SLOPE] =     0x20;			
		   emu_AX12_memory[i][AX12::P_CCW_COMPLIANCE_SLOPE] =    0x20;			
		   emu_AX12_memory[i][AX12::P_PUNCH_L] =                 0x20;			
		   emu_AX12_memory[i][AX12::P_PUNCH_H] =                 0x00;			
      }
	}

	EMULATE_RECORD = false;

	if(logfileAX12 != 0) fclose(logfileAX12);

	logfileAX12 = fopen("simulation_AX12.pos", "w");

	if((logfileAX12 == 0) && (DEBUG_PRINT == true)) fprintf(stderr, "Can not create Simulation file!\n");

	simTime = 0.0;
	simIncrement = 0.008;

	return;
}
// End emulation support

#endif
