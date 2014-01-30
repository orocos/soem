/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : slaveinfo [ifname] [-sdo]
 * Ifname is NIC interface, f.e. eth0.
 * Optional -sdo to display CoE object dictionary.
 *
 * This shows the configured slave data.
 *
 * (c)Arthur Ketels 2010 - 2011
 */

#include <stdio.h>
#include <string.h>

#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatconfig.h"
#include "ethercatcoe.h"
#include "ethercatdc.h"
#include "ethercatprint.h"

char IOmap[4096];
ec_ODlistt ODlist;
ec_OElistt OElist;
boolean printSDO = FALSE;
char usdo[128];
char hstr[1024];

char* SDO2string(uint16 slave, uint16 index, uint8 subidx, uint16 dtype)
{
	int l = sizeof(usdo) - 1, i;
	uint8 *u8;
	int8 *i8;
	uint16 *u16;
	int16 *i16;
	uint32 *u32;
	int32 *i32;
	uint64 *u64;
	int64 *i64;
	float *sr;
	double *dr;
	char es[32];

	memset(&usdo, 0, 128);
	ec_SDOread(slave, index, subidx, FALSE, &l, &usdo, EC_TIMEOUTRXM);
	if (EcatError)
	{
		return ec_elist2string();
	}
	else
	{
		switch(dtype)
		{
			case ECT_BOOLEAN:
				u8 = (uint8*) &usdo[0];
				if (*u8) sprintf(hstr, "TRUE"); 
				 else sprintf(hstr, "FALSE");
				break;
			case ECT_INTEGER8:
				i8 = (int8*) &usdo[0];
				sprintf(hstr, "0x%2.2x %d", *i8, *i8); 
				break;
			case ECT_INTEGER16:
				i16 = (int16*) &usdo[0];
				sprintf(hstr, "0x%4.4x %d", *i16, *i16); 
				break;
			case ECT_INTEGER32:
			case ECT_INTEGER24:
				i32 = (int32*) &usdo[0];
				sprintf(hstr, "0x%8.8x %d", *i32, *i32); 
				break;
			case ECT_INTEGER64:
				i64 = (int64*) &usdo[0];
				sprintf(hstr, "0x%16.16llx %lld", *i64, *i64); 
				break;
			case ECT_UNSIGNED8:
				u8 = (uint8*) &usdo[0];
				sprintf(hstr, "0x%2.2x %u", *u8, *u8); 
				break;
			case ECT_UNSIGNED16:
				u16 = (uint16*) &usdo[0];
				sprintf(hstr, "0x%4.4x %u", *u16, *u16); 
				break;
			case ECT_UNSIGNED32:
			case ECT_UNSIGNED24:
				u32 = (uint32*) &usdo[0];
				sprintf(hstr, "0x%8.8x %u", *u32, *u32); 
				break;
			case ECT_UNSIGNED64:
				u64 = (uint64*) &usdo[0];
				sprintf(hstr, "0x%16.16llx %llu", *u64, *u64); 
				break;
			case ECT_REAL32:
				sr = (float*) &usdo[0];
				sprintf(hstr, "%f", *sr); 
				break;
			case ECT_REAL64:
				dr = (double*) &usdo[0];
				sprintf(hstr, "%f", *dr); 
				break;
			case ECT_BIT1:
			case ECT_BIT2:
			case ECT_BIT3:
			case ECT_BIT4:
			case ECT_BIT5:
			case ECT_BIT6:
			case ECT_BIT7:
			case ECT_BIT8:
				u8 = (uint8*) &usdo[0];
				sprintf(hstr, "0x%x", *u8); 
				break;
			case ECT_VISIBLE_STRING:
				strcpy(hstr, usdo);
				break;
			case ECT_OCTET_STRING:
				hstr[0] = 0x00;
				for (i = 0 ; i < l ; i++)
				{ 
					sprintf(es, "0x%2.2x ", usdo[i]);
					strcat( hstr, es);
				}
				break;
			default:
				sprintf(hstr, "Unknown type");
		}
		return hstr;
	}
}

void slaveinfo(char *ifname)
{
	int cnt, i, j, nSM;
	uint16 ssigen, dtype;
	
	printf("Starting slaveinfo\n");
	
	/* initialise SOEM, bind socket to ifname */
	if (ec_init(ifname))
	{	
		printf("ec_init on %s succeeded.\n",ifname);
		/* find and auto-config slaves */
		if ( ec_config(FALSE, &IOmap) > 0 )
		{
			printf("%d slaves found and configured.\n",ec_slavecount);
			printf("Calculated workcounter %d\n",ec_group[0].expectedWKC);
			/* wait for all slaves to reach SAFE_OP state */
			ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 3);
			if (ec_slave[0].state != EC_STATE_SAFE_OP )
			{
				printf("Not all slaves reached safe operational state.\n");
				ec_readstate();
				for(i = 1; i<=ec_slavecount ; i++)
				{
					if(ec_slave[i].state != EC_STATE_SAFE_OP)
					{
						printf("Slave %d State=%2x StatusCode=%4x : %s\n",
							i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
					}
				}
			}
			
			ec_configdc();
			
			ec_readstate();
			for( cnt = 1 ; cnt <= ec_slavecount ; cnt++)
			{
				printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
					   cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
					   ec_slave[cnt].state, ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
				if (ec_slave[cnt].hasdc) printf(" DCParentport:%d\n", ec_slave[cnt].parentport);
				printf(" Activeports:%d.%d.%d.%d\n", (ec_slave[cnt].activeports & 0x01) > 0 ,
				    								 (ec_slave[cnt].activeports & 0x02) > 0 , 
				    								 (ec_slave[cnt].activeports & 0x04) > 0 , 
				    								 (ec_slave[cnt].activeports & 0x08) > 0 );
				printf(" Configured address: %4.4x\n", ec_slave[cnt].configadr);
				printf(" Man: %8.8x ID: %8.8x Rev: %8.8x\n", (int)ec_slave[cnt].eep_man, (int)ec_slave[cnt].eep_id, (int)ec_slave[cnt].eep_rev);
				for(nSM = 0 ; nSM < EC_MAXSM ; nSM++)
				{
					if(ec_slave[cnt].SM[nSM].StartAddr > 0)
						printf(" SM%1d A:%4.4x L:%4d F:%8.8x Type:%d\n",nSM, ec_slave[cnt].SM[nSM].StartAddr, ec_slave[cnt].SM[nSM].SMlength,
						    	(int)ec_slave[cnt].SM[nSM].SMflags, ec_slave[cnt].SMtype[nSM]);
				}
				for(j = 0 ; j < ec_slave[cnt].FMMUunused ; j++)
				{
					printf(" FMMU%1d Ls:%8.8x Ll:%4d Lsb:%d Leb:%d Ps:%4.4x Psb:%d Ty:%2.2x Act:%2.2x\n", j,
						     (int)ec_slave[cnt].FMMU[j].LogStart, ec_slave[cnt].FMMU[j].LogLength, ec_slave[cnt].FMMU[j].LogStartbit,
						     ec_slave[cnt].FMMU[j].LogEndbit, ec_slave[cnt].FMMU[j].PhysStart, ec_slave[cnt].FMMU[j].PhysStartBit,
						     ec_slave[cnt].FMMU[j].FMMUtype, ec_slave[cnt].FMMU[j].FMMUactive);
				}
				printf(" FMMUfunc 0:%d 1:%d 2:%d 3:%d\n",
				         ec_slave[cnt].FMMU0func, ec_slave[cnt].FMMU2func, ec_slave[cnt].FMMU2func, ec_slave[cnt].FMMU3func);
				printf(" MBX length wr: %d rd: %d MBX protocols : %2.2x\n", ec_slave[cnt].mbx_l, ec_slave[cnt].mbx_rl, ec_slave[cnt].mbx_proto);
				ssigen = ec_siifind(cnt, ECT_SII_GENERAL);
				/* SII general section */
				if (ssigen)
                {
					ec_slave[cnt].CoEdetails = ec_siigetbyte(cnt, ssigen + 0x07);
					ec_slave[cnt].FoEdetails = ec_siigetbyte(cnt, ssigen + 0x08);
					ec_slave[cnt].EoEdetails = ec_siigetbyte(cnt, ssigen + 0x09);
					ec_slave[cnt].SoEdetails = ec_siigetbyte(cnt, ssigen + 0x0a);
					if((ec_siigetbyte(cnt, ssigen + 0x0d) & 0x02) > 0)
					{
						ec_slave[cnt].blockLRW = 1;
						ec_slave[0].blockLRW++;						
					}	
					ec_slave[cnt].Ebuscurrent = ec_siigetbyte(cnt, ssigen + 0x0e);
					ec_slave[cnt].Ebuscurrent += ec_siigetbyte(cnt, ssigen + 0x0f) << 8;
					ec_slave[0].Ebuscurrent += ec_slave[cnt].Ebuscurrent;
                }
				printf(" CoE details: %2.2x FoE details: %2.2x EoE details: %2.2x SoE details: %2.2x\n",
					     ec_slave[cnt].CoEdetails, ec_slave[cnt].FoEdetails, ec_slave[cnt].EoEdetails, ec_slave[cnt].SoEdetails);
				printf(" Ebus current: %d[mA]\n only LRD/LWR:%d\n",
					     ec_slave[cnt].Ebuscurrent, ec_slave[cnt].blockLRW);
				if ((ec_slave[cnt].mbx_proto & 0x04) && printSDO)
				{
					ODlist.Entries = 0;
					memset(&ODlist, 0, sizeof(ODlist));
					if( ec_readODlist(cnt, &ODlist))
					{
						printf(" CoE Object Description found, %d entries.\n",ODlist.Entries);
						for( i = 0 ; i < ODlist.Entries ; i++)
						{
							ec_readODdescription(i, &ODlist); 
							while(EcatError)
							{
								printf("%s", ec_elist2string());
							}
							printf(" Index: %4.4x Datatype: %4.4x Objectcode: %2.2x Name: %s\n",
								   ODlist.Index[i], ODlist.DataType[i], ODlist.ObjectCode[i], ODlist.Name[i]);
							memset(&OElist, 0, sizeof(OElist));
							ec_readOE(i, &ODlist, &OElist);
							while(EcatError)
							{
								printf("%s", ec_elist2string());
							}
							for( j = 0 ; j < ODlist.MaxSub[i]+1 ; j++)
							{
								if ((OElist.DataType[j] > 0) && (OElist.BitLength[j] > 0))
								{
									printf("  Sub: %2.2x Datatype: %4.4x Bitlength: %4.4x Obj.access: %4.4x Name: %s\n",
									   j, OElist.DataType[j], OElist.BitLength[j], OElist.ObjAccess[j], OElist.Name[j]);
									if ((OElist.ObjAccess[j] & 0x0007))
									{
										dtype = OElist.DataType[j];
										printf("          Value :%s\n", SDO2string(cnt, ODlist.Index[i], j, OElist.DataType[j]));
									}
								}
							}	
						}	
					}
					else
					{
						while(EcatError)
						{
							printf("%s", ec_elist2string());
						}
					}
				}	
			}	
		}
		else
		{
			printf("No slaves found!\n");
		}
		printf("End slaveinfo, close socket\n");
		/* stop SOEM, close socket */
		ec_close();
	}
	else
	{
		printf("No socket connection on %s\nExcecute as root\n",ifname);
	}	
}	

int main(int argc, char *argv[])
{
	printf("SOEM (Simple Open EtherCAT Master)\nSlaveinfo\n");
	
	if (argc > 1)
	{		
		if ((argc > 2) && (strncmp(argv[2], "-sdo", sizeof("-sdo")) == 0)) printSDO = TRUE;
		/* start slaveinfo */
		slaveinfo(argv[1]);
	}
	else
	{
		printf("Usage: slaveinfo ifname [options]\nifname = eth0 for example\nOptions : -sdo : print SDO info\n");
	}	
	
	printf("End program\n");
	return (0);
}
