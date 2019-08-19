#include "Dcm2_JSEB2.h"
#include "adcjmultix.h"

//#include <Dcm2_WinDriver.h>
//#include <Dcm2_FileBuffer.h>
#include <iostream>
#include <fstream>

//#include <ogzBuffer.h>
//#include <olzoBuffer.h>
#include <oBuffer.h>
#include <phenixTypes.h>
#include <packetConstants.h>
#include <EventTypes.h>
#include <oEvent.h>

using namespace std;
extern bool exitRequested;

#define FAILURE 1
/**
 * this code is adopted from sphenix_adc_test_xmit_dcm.cc in
 * scampbel/source/adc_code. This will configure and get the dititizer
 * to write a data file
 * This uses the jseb2_lib in WinDriver/my_projects which talks to the
 * jseb2 card via generated code. This will use MM Dcm2_JSEB2
 * code for all writes. It is to test using the JSEB2 in a threaded environment
 */


ADC::ADC()
{
  dwOptions_send = DMA_TO_DEVICE;
  dwOptions_send = DMA_TO_DEVICE | DMA_ALLOW_CACHE;
  dwOptions_rec = DMA_FROM_DEVICE | DMA_ALLOW_64BIT_ADDRESS;
  adcJseb2Name = string("JSEB2.10.0"); // default adc JSEB2
  obfilename = "/home/phnxrc/desmond/daq/prdf_output/adceventdata.prdf";
  nread = 4096*2+6; /*16384 32768, 65536+4;  number of byte to be readout */
  ifr=0;
  iwrite =0;
  iprint =0;
  icheck =0;
  istop=0;
  tim.tv_sec = 0;
  //    tim.tv_nsec =128000;
  tim.tv_nsec =172000;
  InitJseb2Dev();
  l1_delay = 48;
  trport2 = 2; // port 2 for adc jseb2 access
  activejseb2port = trport2; // default to port 2
  nsample = 31; // default sample size
  _num_xmitgroups = 1;
}


ADC::~ADC()
{
  //delete controller;
  //windriver->Exit();
  //stopWinDriver();
}

void ADC::setJseb2Port(int portnumber)
{
  activejseb2port = portnumber;
  trport2 = portnumber; // temp use of the original variable

} // setJseb2Port

void ADC::InitJseb2Dev()
{
  // use JSEB2 only for now
  //jseb2d = new Jseb2Device();
  //jseb2d->Init(); // allocate DMA buffers
 // is this necessary to use the same mutex; try separate first
 //jseb2d->SetID("JSEB2:10.0"); // set mutex id to dcm2 to both jsebs use the same mutex 
}

void ADC::stopWinDriver()
{

  //cout << "ADC::stopWinDriver called " << endl;
  wdc_started = 0;
  delete controller;
  windriver->Exit();
} // stopWinDriver
/**
 * startWinDriver
 * open WinDriver, create Dcm2_Controller ; get jseb2 devices get Dcm2_JSEB2 object
 * of jseb2 named in arguement
 *
 */
bool ADC::startWinDriver()
{
 
  wdc_started = 1;
  //cout << "ADC:startWinDriver - Open WinDriver " << endl;
  static int ifr = 0;
  ifr = 0;
 // create windriver and controller object
 // use different driver open 2 windrivers
 windriver = Dcm2_WinDriver::instance();
 windriver->SetVerbosity(verbosity);
  if(!windriver->Init())
    {
      cerr << "   JSEB2() - Failed to load WinDriver Library" << endl;
      windriver->Exit();
      return false;
    }    
 

  controller = new Dcm2_Controller();
  controller->SetVerbosity(verbosity);
  if(!controller->Init())
    {
      cerr << "   JSEB2() - Failed to initialize the controller machine" << endl;
      delete controller;
      windriver->Exit();
      return false;
    }
 unsigned int njseb2s = controller->GetNJSEB2s();
 //cout << "   A scan of the PCIe bus shows " << njseb2s << " JSEB2(s) installed" << endl;
 //cout << "get  jseb2 for JSEB2 device " << adcJseb2Name.c_str()<< endl;
  jseb2 = controller->GetJSEB2(adcJseb2Name);
  // get the handle to the jseb
  jseb2->Init();
  //cout << "ADC:   " << jseb2->GetID() << " has been selected and initialized" << endl;
 hDev = jseb2->GetHandle();
 //delete controller;

 
 return true;

} // startWinDriver


void ADC::setObDatafileName(string datafilename)
{
  obfilename = datafilename;
  //cout << "set obdata filename = " << obfilename.c_str() << endl;
} // setObDatafileName

/**
 * freeDMAbuffer
 * free the DMCContigBuf allocated by WDC_DMAContigBufLock
 */
void ADC::freeDMAbuffer()
{
  /*
DWORD DLLCALLCONV WDC_DMASGBufLock(
    WDC_DEVICE_HANDLE hDev,
    PVOID pBuf,
    DWORD dwOptions,
    DWORD dwDMABufSize,
    WD_DMA **ppDma);
  */

  //WDC_DMASGBufLock(hDev,pbuf_send,dwOptions_send, dwDMABufSize, &pDma_send);
  WDC_DMABufUnlock(pDma_send);
} // freeDMAbuffer

void ADC::setNumXmitGroups(int nxmitgroups)
{
  _num_xmitgroups = nxmitgroups;
} // setNumXmitGroups

// set the number of digitizer boards in an xmit group
void ADC::setNumAdcBoards(int xmitgroup, int numboards)
 {
   nmod = numboards;
   _xmit_groups[xmitgroup].num_dig_boards = numboards;

 } // setNumAdcBoards

// set slot number of first adc module
void ADC::setFirstAdcSlot(int xmitgroup,int slotnumber)
{
  imod_start = slotnumber;
  _xmit_groups[xmitgroup].firstdigslot_nm = slotnumber;
} // setFirstAdcSlot

void ADC::setSampleSize(int xmitgroup,int samplesize)
{
  nsample = samplesize;
  _xmit_groups[xmitgroup].nsamples = samplesize;

} // setSampleSize

// initializes 1 adc board and 1 xmit board located in the slot after the adc board
bool ADC::initialize(UINT32 eventstotake)
{

  bool windriverstatus = false;
  cout << __FILE__ << " ADC::initialize " << endl;
  int   adc_data[64][32];
  int   ixgroup = 0; // xmit group
  nread = 4096*2+6; /*16384 32768, 65536+4;  number of byte to be readout */
  ifr=0;
  iwrite =0;
  iprint =0;
  icheck =0;
  istop=0;
  isel_dcm = 0;
  isel_xmit = 1; // always use xmit board
  iext_trig = 1;

  windriverstatus =  startWinDriver();
  if (windriverstatus == false)
    {
      return false;
    }

  nevent = eventstotake;
 cout << __FILE__ << " ADC set to use DMA " << endl;

  jseb2->SetUseDMA(true); // write one word

  px = buf_send;
  py = read_array;
  // set xmit slot

  
  ichip=6;

     // here f0000008 is sent to tx_mode_reg 
     // skipping initialize tx register

   ifr=0;
     buf_send[0]=0x0;
     buf_send[1]=0x0;
     i=1;
     k=1;
     //i = pcie_send_1(hDev, i, k, px); //pcie_send_1 is for adc operations port 2
     // i = 1 == DMA; i = 0 == single word
     jseb2->Write(trport2,k,px);
     cout << __FILE__ << " Configure all Xmit groups " << endl;
// configure all xmit groups
     for (ixgroup = 0; ixgroup < _num_xmitgroups ; ixgroup++ )
       {
         cout << __FILE__ << " Configure Xmit group " << ixgroup << endl;
         imod_start = _xmit_groups[ixgroup].firstdigslot_nm;
         nmod = _xmit_groups[ixgroup].num_dig_boards;
         imod_xmit = imod_start+nmod;
         //imod_xmit = _xmit_groups[ixgroup].firstdigslot_nm; // 08/12/2019 set to slot of xmit board not 1st mod
         cout << __FILE__ << " reset of  slot " << imod_start << " num of boards " << nmod << endl;
         ichip = sp_xmit_sub; //1
         //analog reset of xmit
         buf_send[0]=(imod_xmit <<11)+ (ichip << 8) + sp_xmit_rxanalogreset + (0<<16) ;
         i= 1; // dma
         k= 1;
         //i = pcie_send_1(hDev, i, k, px);
         jseb2->Write(trport2,k,px); // write 1 word dma mode
         usleep(10);

         //digital reset of xmit
         buf_send[0]=(imod_xmit <<11)+ (ichip << 8) + sp_xmit_rxdigitalreset + (0<<16) ;
         i= 1;
         k= 1;
         jseb2->Write(trport2,k,px); // write 1 word dma mode

         //set byte ordinger
         buf_send[0]=(imod_xmit <<11)+ (ichip << 8) + sp_xmit_rxbytord + (0<<16) ;
         i= 1;
         k= 1;
         jseb2->Write(trport2,k,px); // write 1 word dma mode

         //init xmit
         buf_send[0]=(imod_xmit <<11)+ (ichip << 8) + sp_xmit_init + (0<<16) ;
         //mod_xmit+dcm_last+1, ichip=1, sp_xmit_init=4
         i= 1;
         k= 1;
         // i = pcie_send_1(hDev, i, k, px);
         jseb2->Write(trport2,k,px); // write 1 word dma mode
         usleep(10);
         //} // next xmit board group

     //   send init to the controller -- take adc controller out of 'run' state
     buf_send[0]= (0x2<<8)+sp_cntrl_timing+ (sp_cntrl_init<<16);
     i= 1;
     k= 1;
     //i = pcie_send_1(hDev, i, k, px);
    jseb2->Write(trport2,k,px); // write 1 word dma mode
    usleep(10);

     buf_send[0]= (0x2<<8)+sp_cntrl_timing+ (sp_cntrl_init<<16);
     i= 1;
     k= 1;
     //i = pcie_send_1(hDev, i, k, px);
     jseb2->Write(trport2,k,px); // write 1 word dma mode
     usleep(10);

     //   send reset to the controller -- put adc controller back in 'run' state
     buf_send[0]= (0x2<<8)+ sp_cntrl_timing+ (sp_cntrl_reset<<16);
     i= 1;
     k= 1;
     //i = pcie_send_1(hDev, i, k, px);
    jseb2->Write(trport2,k,px); // write 1 word dma mode
     usleep(10);


     // configure each module of each xmit group
     //for (ixgroup = 0; ixgroup < _num_xmitgroups ; ixgroup++ )
     // {
     //    nmod = _xmit_groups[ixgroup].num_dig_boards;
     //     imod_start = _xmit_groups[ixgroup].firstdigslot_nm;
         //imod_xmit = imod_start+nmod; // set location of xmit for this xmit group 8/12/2019
         for (ik =0; ik<nmod; ik++) 
           {
             //cout << __FILE__ << " Configure module " << ik << " of Xmit group " << ixgroup << endl;
             // get module slot number
             imod = ik+imod_start;
             //imod = ik + _xmit_groups[ixgroup].firstdigslot_nm;
             // send sample size
             ichip = sp_adc_slowcntl_sub;
             //buf_send[0]=(imod <<11)+ (ichip << 8) + sp_adc_evt_sample + ((nsample-1)<<16) ;
             buf_send[0]=(imod <<11)+ (ichip << 8) + sp_adc_evt_sample + ((_xmit_groups[ixgroup].nsamples -1 )<<16);
             i= 1;
             k= 1;
             //i = pcie_send_1(hDev, i, k, px);
             jseb2->Write(trport2,k,px); // write 1 word dma mode
             usleep(10);
             //cout << __FILE__ << " set Sample Size to  " << _xmit_groups[ixgroup].nsamples << endl;
             //   set L1 delay  -- to 255
             buf_send[0]=(imod <<11)+ (ichip << 8) + sp_adc_l1_delay + (l1_delay<<16) ;
             //printf(" buf_send = %x\n", buf_send[0]);
             i= 1;
             k= 1;
             //i = pcie_send_1(hDev, i, k, px);
             jseb2->Write(trport2,k,px); // write 1 word dma mode
             usleep(10);

             //   set dat2link on for xmit 
             buf_send[0]=(imod <<11)+ (ichip << 8) + sp_adc_rd_link + (1<<16) ; //1 on, 0 off
             i= 1;
             k= 1;
             //i = pcie_send_1(hDev, i, k, px);
             jseb2->Write(trport2,k,px); // write 1 word dma mode
             usleep(10);

             //   set dat2control off for xmit
             buf_send[0]=(imod <<11)+ (ichip << 8) + sp_adc_rd_cntrl + (0x0<<16) ;
             i= 1;
             k= 1;
             jseb2->Write(trport2,k,px); // write 1 word dma mode
             usleep(10);

             //   set pulse trigger off
             buf_send[0]=(imod <<11)+ (ichip << 8) + sp_adc_sel_pulse + (0x0<<16) ;
             i= 1;
             k= 1;
             //i = pcie_send_1(hDev, i, k, px);
             jseb2->Write(trport2,k,px); // write 1 word dma mode
             usleep(10);

             // Here the calibration trigger is set to off when using the xmit.
             buf_send[0]=(imod <<11)+ (ichip << 8) + sp_adc_sel_caltrig + (0x0<<16) ;
             i= 1;
             k= 1;
             //i = pcie_send_1(hDev, i, k, px);
             jseb2->Write(trport2,k,px); // write 1 word dma mode
             usleep(10);

             //   set test trigger off
             buf_send[0]=(imod <<11)+ (ichip << 8) + sp_adc_sel_test_trig + (0x0<<16) ;
             i= 1;
             k= 1;
             //i = pcie_send_1(hDev, i, k, px);
             jseb2->Write(trport2,k,px); // write 1 word dma mode
             usleep(10);

             //   set select L1 trigger on
             buf_send[0]=(imod <<11)+ (ichip << 8) + sp_adc_sel_l1 + (0x1<<16) ;
             i= 1;
             k= 1;
             //i = pcie_send_1(hDev, i, k, px);
             jseb2->Write(trport2,k,px); // write 1 word dma mode
             usleep(10);

             // here is a test if imod != imod_start then tunr off link_rxoff
             // This should not be executed if there is only 1 module
             if(imod != imod_start) 
               {
                 //printf(" turn link rx off if not first module");

                 ichip = sp_adc_slowcntl_sub;
                 buf_send[0]=(imod <<11)+ (ichip << 8) + sp_adc_sel_link_rxoff + (0<<16) ;
                 i= 1;
                 k= 1;
                 //i = pcie_send_1(hDev, i, k, px);
                 jseb2->Write(trport2,k,px); // write 1 word dma mode

                 usleep(10);
                 //printf(" rx_off called , module %d\n", imod);
               } //if imod!= imod_start


             i = adc_setup(hDev,imod, 0);
             usleep(500);
           } // next module loop over nmod
         //} // next xmit group 08/12/2019 include last module location for each xmit group

  // set last module location from xmit
 
	 imod = imod_xmit;
	 ichip = sp_xmit_sub;
	 buf_send[0]=(imod <<11)+ (ichip << 8) + sp_xmit_lastmod + (imod_start<<16) ;
	 i= 1;
	 k= 1;
	 //i = pcie_send_1(hDev, i, k, px);
	 jseb2->Write(trport2,k,px); // write 1 word dma mode
	 usleep(10);
	 printf(" set last module location for xmit %d to %d \n", imod, imod_start);

     // mored sp_adc_readback_sub command  to here within xmit group loop

      ichip = sp_adc_readback_sub ;   // controller data go to ADC input section
      buf_send[0]=(imod<<11)+(ichip<<8)+(8) + (0x0<<16);  // 8 = "nothing" (grabbing the backplane bus)
      i=1;
      k=1;
      //i = pcie_send_1(hDev, i, k, buf_send);
      jseb2->Write(trport2,k,px); // write 1 word dma mode
      usleep(10);

         } // next xmit group 8/12/2019 include setting last module location from each xmit group

     // LOWER THE BUSY
      buf_send[0]= (0x2<<8)+sp_cntrl_busyrst+ (0<<16);
      i= 1;
      k= 1;
      //i = pcie_send_1(hDev, i, k, buf_send);
      jseb2->Write(trport2,k,px); // write 1 word dma mode
      usleep(10);

      // more sp_adc_readback_sub command from here to within xmit group loop





    // close windriver
    stopWinDriver();

  return true;
} // initialize



void ADC::setJseb2Name(string jseb2name)
{
  adcJseb2Name = jseb2name;
} // setJseb2Name
void ADC::adcResets(int index_nmod)
{

  printf("Execute pulse generator code \n");

} // adcResets


void ADC::setBusy()
{
     buf_send[0]= (0x2<<8)+sp_cntrl_busyrst+ (0<<16);
      i= 1;
      k= 1;
      //i = pcie_send_1(hDev, i, k, buf_send);
      jseb2->Write(trport2,k,px);
      usleep(10);

      ichip = sp_adc_readback_sub ;   // controller data go to ADC input section
      buf_send[0]=(imod<<11)+(ichip<<8)+(8) + (0x0<<16);  // 8 = "nothing" (grabbing the backplane bus)
      i=1;
      k=1;
      //i = pcie_send_1(hDev, i, k, buf_send);
      jseb2->Write(trport2,k,px);
      usleep(10);


} // setBusy



/**
 * setDcm2State
 * could be used used to download specific dcm2 state condistions: e.g. mask values
 */
void setDcm2State()
{


} // setDcm2State








void ADC::setL1Delay(int l1delay)
 {
    l1_delay = l1delay;
 }






// adc_setup
int ADC::adc_setup(WDC_DEVICE_HANDLE hDev, int imod, int iprint)
{
#define  sp_cntrl_timing            3
#define  sp_cntrl_init           0x30
#define  sp_cntrl_reset          0x28
#define  sp_cntrl_l1             0x24
#define  sp_cntrl_pulse          0x22

#define  sp_adc_readback_sub        4
#define  sp_adc_readback_transfer   1
#define  sp_adc_readback_read       2
#define  sp_adc_readback_status     3
#
#define  sp_adc_input_sub           2
#define  sp_adc_slowcntl_sub        1

#define  sp_adc_l1_delay            1
#define  sp_adc_evt_sample          2

#define  sp_adc_rd_link             3
#define  sp_adc_rd_cntrl            4

#define  sp_adc_sel_l1              5
#define  sp_adc_sel_pulse           6
#define  sp_adc_sel_test_trig       7

#define  sp_adc_sel_linux_rxoff     8

#define  sp_adc_u_adc_align        10
#define  sp_adc_l_adc_align        11
#define  sp_adc_pll_reset          12

#define  sp_adc_rstblk             13
#define  sp_adc_test_pulse         14

#define  sp_adc_dpa_reset          15

#define  sp_adc_spi_add            20
#define  sp_adc_spi_data           30

#define  sp_adc_lnk_tx_dreset      20
#define  sp_adc_lnk_tx_areset      21
#define  sp_adc_trg_tx_dreset      22
#define  sp_adc_trg_tx_areset      23
#define  sp_adc_lnk_rx_dreset      24
#define  sp_adc_lnk_rx_areset      25

#define  sp_adc_link_mgmt_reset    26
#define  sp_adc_link_conf_w        27
#define  sp_adc_link_conf_add      30
#define  sp_adc_link_conf_data_l   31
#define  sp_adc_link_conf_data_u   32

#define  sp_xmit_lastmod            1
#define  sp_xmit_rxanalogreset      2
#define  sp_xmit_rxdigitalreset     3
#define  sp_xmit_init               4

#define  sp_xmit_sub                1


    int ichip,ich,i,k;
    UINT32 buf_send[40000];
    UINT32 *px;



    px = buf_send;
    ichip = sp_adc_input_sub ;   // controller data go to ADC input section
    for (ich=0; ich<8; ich++) {

//    set spi address

     buf_send[0]=(imod <<11)+ (ichip << 8) + sp_adc_spi_add + (ich<<16) ;
     i= 1;
     k= 1;
     //i = pcie_send_1(hDev, i, k, px);
     jseb2->Write(trport2,k,px); // write 1 word dma mode
//
     if(iprint==1) printf(" set spi address to channel %d, module %d\n", ich, imod);
//       scanf("%d",&is);

//      buf_send[0]=(imod<<11)+(ichip<<8)+(mb_pmt_spi_)+((is & 0xf)<<16); //set spi address
//      i=1;
//        k=1;
//        i = pcie_send(hDev, i, k, px);
//       printf(" spi port %d \n",is);
//       scanf("%d",&ik);

//        imod=11;
//        ichip=5;
//
//
//     power reset the ADC
//
     buf_send[0]=(imod<<11)+(ichip<<8)+(sp_adc_spi_data)+(0x0300<<16); //1st next word will be overwrite by the next next word
     buf_send[1]=(((0x0)<<13)+(0x8))+((0x03)<<24)+((0x0)<<16);
//
//  set /w =0, w1,w2 =0, a12-a0 = 0x8, data =0x03;
//

     i=1;
     k=2;

     //i = pcie_send_1(hDev, i, k, px);
     jseb2->Write(trport2,k,px); // write 1 word dma mode
     usleep(100);   // sleep for 100us
//
//     reove power reset
//
     buf_send[0]=(imod<<11)+(ichip<<8)+(sp_adc_spi_data)+(0x0300<<16); //1st next word will be overwrite by the next next word
     buf_send[1]=(((0x0)<<13)+(0x8))+((0x00)<<24)+((0x0)<<16);
//
//  set /w =0, w1,w2 =0, a12-a0 = 0x8, data =0x00;
//

     i=1;
     k=2;
//       i = pcie_send(hDev, i, k, px);
     //i = pcie_send_1(hDev, i, k, px);
     jseb2->Write(trport2,k,px); // write 1 word dma mode
     usleep(100);   // sleep for 100us
//
//     reset ADC
//
     buf_send[0]=(imod<<11)+(ichip<<8)+(sp_adc_spi_data)+(0x0300<<16); //1st next word will be overwrite by the next next word
     buf_send[1]=(((0x0)<<13)+(0x0))+((0x3c)<<24)+((0x0)<<16);
//
//  set /w =0, w1,w2 =0, a12-a0 = 0x0, data =0x3c;
//

     i=1;
     k=2;
//       i = pcie_send(hDev, i, k, px);
     //i = pcie_send_1(hDev, i, k, px);
     jseb2->Write(trport2,k,px); // write 1 word dma mode
     usleep(100);   // sleep for 100us
//
//       if(ich ==7) {
     if(iprint ==1) printf(" send last command ich= %d module %d\n", ich, imod);
//        scanf("%d",&is);
//       }
//
//    set LVDS termination  set addess 0x15  to 1 to 2x drive
//
     buf_send[0]=(imod<<11)+(ichip<<8)+(sp_adc_spi_data)+(0x0300<<16); //1st next word will be overwrite by the next next word
     buf_send[1]=(((0x0)<<12)+(0x15))+((0x20)<<24)+((0x0)<<16);
//
//  set /w =0, w1,w2 =0, a12-a0 = 0x15, data =0x1;
//
     i=1;
     k=2;
//       i = pcie_send(hDev, i, k, px);
     //i = pcie_send_1(hDev, i, k, px);
     jseb2->Write(trport2,k,px); // write 1 word dma mode
     usleep(100);   // sleep for 100us

//
//    set fix pattern  0xa = sync pattern
//
     buf_send[0]=(imod<<11)+(ichip<<8)+(sp_adc_spi_data)+(0x0300<<16); //1st next word will be overwrite by the next next word
     buf_send[1]=(((0x0)<<13)+(0xd))+((0xa)<<24)+((0x0)<<16);
//
//  set /w =0, w1,w2 =0, a12-a0 = 0xd, data =0xa;
//

     i=1;
     k=2;
//      i = pcie_send(hDev, i, k, px);
     //i = pcie_send_1(hDev, i, k, px);
     jseb2->Write(trport2,k,px); // write 1 word dma mode
     usleep(100);   // sleep for 100us
//
//       printf(" type 1 to continue \n");
//       scanf("%d",&is);
    }


    usleep(100);
//
//    set up for ADC alignment
//
    ichip = sp_adc_slowcntl_sub ;   // controller data go to ADC input section
//
//       pll reset
//
    if(iprint == 1)printf(" send pll reset\n");
//      scanf("%d",&is);
    buf_send[0]=(imod <<11)+ (ichip << 8) + sp_adc_pll_reset + (0<<16) ;
    i= 1;
    k= 1;
    //i = pcie_send_1(hDev, i, k, px);
     jseb2->Write(trport2,k,px); // write 1 word dma mode
    usleep(1000);

//
//       DPA reset
//
    if(iprint == 1) printf(" send ADC DPA reset module %d\n", imod);
//    scanf("%d",&is);
    buf_send[0]=(imod <<11)+ (ichip << 8) + sp_adc_dpa_reset + (0<<16) ;
    i= 1;
    k= 1;
    //i = pcie_send_1(hDev, i, k, px);
     jseb2->Write(trport2,k,px); // write 1 word dma mode
    usleep(1000);

//
//       upper ADC alignment
//
    if(iprint==1) printf(" send upper channel align module %d \n", imod);
//    scanf("%d",&is);
    buf_send[0]=(imod <<11)+ (ichip << 8) + sp_adc_u_adc_align + (0<<16) ;
    i= 1;
    k= 1;
    //i = pcie_send_1(hDev, i, k, px);
     jseb2->Write(trport2,k,px); // write 1 word dma mode
    usleep(1000);
//
//       lower ADC alignment
//
    if(iprint == 1) printf(" send lower channel align module %d\n", imod);
//      scanf("%d",&is);
    buf_send[0]=(imod <<11)+ (ichip << 8) + sp_adc_l_adc_align + (0<<16) ;
    i= 1;
    k= 1;
    //i = pcie_send_1(hDev, i, k, px);
     jseb2->Write(trport2,k,px); // write 1 word dma mode
    usleep(1000);
//
//
//

   ichip = sp_adc_input_sub ;   // controller data go to ADC input section
   for (ich=0; ich<8; ich++) {
    buf_send[0]=(imod <<11)+ (ichip << 8) + sp_adc_spi_add + (ich<<16) ;
    i= 1;
    k= 1;
    //i = pcie_send_1(hDev, i, k, px);
     jseb2->Write(trport2,k,px); // write 1 word dma mode
    usleep(100);
//
//    output offset binary code
//
    buf_send[0]=(imod<<11)+(ichip<<8)+(sp_adc_spi_data)+(0x0300<<16); //1st next word will be overwrite by the next next word
    buf_send[1]=(((0x0)<<12)+(0x14))+((0x0)<<24)+((0x0)<<16);
//
//  set /w =0, w1,w2 =0, a12-a0 = 0x14, data =0x0;
//

    i=1;
    k=2;

    //i = pcie_send_1(hDev, i, k, px);
     jseb2->Write(trport2,k,px); // write 1 word dma mode
    usleep(100);   // sleep for 100us

//
//    unset fix pattern  0x0 for normal data taking   --- set to test condition
//
    buf_send[0]=(imod<<11)+(ichip<<8)+(sp_adc_spi_data)+(0x0300<<16); //1st next word will be overwrite by the next next word
    buf_send[1]=(((0x0)<<13)+(0xd))+((0x0)<<24)+((0x0)<<16);
//
//  set /w =0, w1,w2 =0, a12-a0 = 0xd, data =0x0;
//

    i=1;
    k=2;

    //i = pcie_send_1(hDev, i, k, px);
     jseb2->Write(trport2,k,px); // write 1 word dma mode
    usleep(100);   // sleep for 100us
   }
//
//  test routine
//
//    set spi address

    ich =0;
    buf_send[0]=(imod <<11)+ (ichip << 8) + sp_adc_spi_add + (ich<<16) ;
    i= 1;
    k= 1;
//    i = pcie_send_1(hDev, i, k, px);
    usleep(100);
//
//    remove channel H,G,F,E from the selection list
//
    buf_send[0]=(imod<<11)+(ichip<<8)+(sp_adc_spi_data)+(0x0300<<16); //1st next word will be overwrite by the next next word
    buf_send[1]=(((0x0)<<13)+(0x4))+((0x0)<<24)+((0x0)<<16);
//
//  set /w =0, w1,w2 =0, a12-a0 = 0x4, data =0x0;
//

    i=1;
    k=2;

//    i = pcie_send_1(hDev, i, k, px);
    usleep(100);   // sleep for 100us
//
//    only keep channel a from the selection list
//
    buf_send[0]=(imod<<11)+(ichip<<8)+(sp_adc_spi_data)+(0x0300<<16); //1st next word will be overwrite by the next next word
    buf_send[1]=(((0x0)<<13)+(0x5))+((0x1)<<24)+((0x0)<<16);
//
//  set /w =0, w1,w2 =0, a12-a0 = 0x5, data =0x1;
//

    i=1;
    k=2;

///    i = pcie_send_1(hDev, i, k, px);
    usleep(100);   // sleep for 100us
//
//    set fix pattern  0xa = sync pattern
//
     buf_send[0]=(imod<<11)+(ichip<<8)+(sp_adc_spi_data)+(0x0300<<16); //1st next word will be overwrite by the next next word
     buf_send[1]=(((0x0)<<13)+(0xd))+((0xc)<<24)+((0x0)<<16);
//
//  set /w =0, w1,w2 =0, a12-a0 = 0xd, data =0xc;
//

     i=1;
     k=2;
//      i = pcie_send(hDev, i, k, px);
//     i = pcie_send_1(hDev, i, k, px);
     usleep(100);   // sleep for 100us
//
//    restore selection list 1
//
    buf_send[0]=(imod<<11)+(ichip<<8)+(sp_adc_spi_data)+(0x0300<<16); //1st next word will be overwrite by the next next word
    buf_send[1]=(((0x0)<<13)+(0x4))+((0xf)<<24)+((0x0)<<16);
//
//  set /w =0, w1,w2 =0, a12-a0 = 0x4, data =0x0;
//

    i=1;
    k=2;

//    i = pcie_send_1(hDev, i, k, px);
    usleep(100);   // sleep for 100us
//
//    restore selection list 2
//
    buf_send[0]=(imod<<11)+(ichip<<8)+(sp_adc_spi_data)+(0x0300<<16); //1st next word will be overwrite by the next next word
    buf_send[1]=(((0x0)<<13)+(0x5))+((0x3f)<<24)+((0x0)<<16);
//
//  set /w =0, w1,w2 =0, a12-a0 = 0x5, data =0x1;
//

    i=1;
    k=2;

//    i = pcie_send_1(hDev, i, k, px);
    usleep(100);   // sleep for 100us



    return i;
} // adc_setup


