//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2011
// European Synchrotron Radiation Facility
// BP 220, Grenoble 38043
// FRANCE
//
// This is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//###########################################################################
//
// DhyanaCamera.h
// Created on: August 09, 2023
// Author: Marc DESGRANGES

#ifndef DHYANA6060CAMERA_H_
#define DHYANA6060CAMERA_H_


#include <ostream>
#include <map>
#include <process.h>
#include "Dhyana6060Compatibility.h"
#include "lima/HwBufferMgr.h"
#include "lima/HwInterface.h"
#include "lima/Debug.h"
#include "lima/Timer.h"
#include "TUCamApi.h"
#include "TUDefine.h"


namespace lima
{
namespace Dhyana6060
{

const int PIXEL_SIZE_WIDTH_MICRON  = 10; // pixel size is 10 micron
const int PIXEL_SIZE_HEIGHT_MICRON = 10; // pixel size is 10 micron

class BufferCtrlObj;
class CSoftTriggerTimer;

/*******************************************************************
 * \class Camera
 * \brief object controlling the Dhyana camera
 *******************************************************************/
class LIBDHYANA6060_API Camera
{
    DEB_CLASS_NAMESPC(DebModCamera, "Camera", "Dhyana6060");

public:

    enum Status
    {
        Ready, Exposure, Readout, Latency, Fault
    } ;

    enum TucamTriggerMode
    {
      kTriggerStandard = TUCCM_TRIGGER_STANDARD,
      kTriggerSynchronous = TUCCM_TRIGGER_SYNCHRONOUS,
      kTriggerGlobal = TUCCM_TRIGGER_GLOBAL,
      //TriggerSoftware = TUCCM_TRIGGER_SOFTWARE, do not map, this mode is used for Lima IntTrigSingle and Timer to retrig
    };

    enum TucamTriggerEdge
    {
      kEdgeRising = TUCTD_RISING,
      kEdgeFalling = TUCTD_FAILING,
    };

    enum TucamSignal
    {
      kSignalTrigIn = TUOPT_IN,        //copy of the trigger IN
      kSignalStart = TUOPT_EXPSTART,   // Exposure start (rolling)
      kSignalGlobal = TUOPT_EXPGLOBAL, // Global exposure
      kSignalReadEnd = TUOPT_READEND   // readout end
    };
    enum TucamSignalEdge
    {
      kSignalEdgeRising = TUOPT_RISING,
      kSignalEdgeFalling = TUOPT_FAILING,
    };

    enum TucamGain
    {
      kGainHDR  = TUGAIN_HDR,
      kGainHigh = TUGAIN_HIGH,
      kGainLow  = TUGAIN_LOW
    };

    Camera(unsigned short timer_period_ms);
    virtual ~Camera();

    void init();
    void reset();
    void prepareAcq();
    void startAcq();
    void stopAcq();
    void getStatus(Camera::Status& status);
    int  getNbHwAcquiredFrames();

    // -- detector info object
    void getImageType(ImageType& type);
    void setImageType(ImageType type);

    void getDetectorType(std::string& type);
    void getDetectorModel(std::string& model);
    void getDetectorImageSize(Size& size);
    void getPixelSize(double& sizex, double& sizey);

    // -- Buffer control object
    HwBufferCtrlObj* getBufferCtrlObj();

    //-- Synch control object
    void setTrigMode(TrigMode mode);
    void getTrigMode(TrigMode& mode);
    bool checkTrigMode(TrigMode mode);

    void setExpTime(double exp_time);
    void getExpTime(double& exp_time);

    void setLatTime(double lat_time);
    void getLatTime(double& lat_time);

    void getExposureTimeRange(double& min_expo, double& max_expo) const;
    void getLatTimeRange(double& min_lat, double& max_lat) const;

    void setNbFrames(int nb_frames);
    void getNbFrames(int& nb_frames);

    //-- Related to Bin control object
    void setBin(const Bin& bin);
    void getBin(Bin& bin);
    void checkBin(Bin& bin);

    //-- Related to Roi control object
    void checkRoi(const Roi& set_roi, Roi& hw_roi);
    void setRoi(const Roi& set_roi);
    void getRoi(Roi& hw_roi);

    ///////////////////////////////
    // -- dhyana6060 specific functions
    ///////////////////////////////

    void getCameraTemperature(double& temp);
    void setSensorTemperatureTarget(double temp);
    void getSensorTemperatureTarget(double& temp);
    void getSensorTemperature(double& temp);
    void setSensorCoolingType(unsigned type);
    void getSensorCoolingType(unsigned& type);
    void setFanSpeed(unsigned speed);
    void getFanSpeed(unsigned& speed);
    void setFanType(unsigned type);
    void getFanType(unsigned& type);
    void setGlobalGain(unsigned gain);
    void getGlobalGain(unsigned& gain);
    void getTucamVersion(std::string& version);
    bool isAcqRunning() const;

    void getFPS(double& fps);	
    void getTriggerMode(TucamTriggerMode& mode);
    void setTriggerMode(TucamTriggerMode mode);
    void getTriggerEdge(TucamTriggerEdge& edge);
    void setTriggerEdge(TucamTriggerEdge edge);
    void setTrigInEdge(unsigned edge);
    void getTrigInEdge(unsigned& edge);
    void setTrigInExpType(unsigned type);
    void getTrigInExpType(unsigned& type);
    void setTrigInDelay(double delay);
    void getTrigInDelay(double& delay);
    void setTrigOutputPort(unsigned port);
    void getTrigOutputPort(unsigned& port);
    void setTrigOutputKind(unsigned kind);
    void getTrigOutputKind(unsigned& kind);
    void setTrigOutputWidth(double width);
    void getTrigOutputWidth(double& width);
    void setTrigOutputDelay(double delay);
    void getTrigOutputDelay(double& delay);
    void setTrigOutputEdge(unsigned edge);
    void getTrigOutputEdge(unsigned& edge);

    void setSensorCooling(unsigned type);
    void getSensorCooling(unsigned& type);

    std::string getAllParameters();
    std::string getParameter(std::string parameter);
    void setParameter(std::string parameter, double value);


    //TUCAM stuff, use TUCAM notations !
    TUCAM_INIT          m_itApi; // TUCAM handle Api
    TUCAM_OPEN          m_opCam; // TUCAM handle camera
    TUCAM_FRAME         m_frame; // TUCAM frame structure
    HANDLE              m_hThdEvent; // TUCAM handle event   
private:
 //read/copy frame
    bool readFrame(void *bptr, int& frame_nb);
    void setStatus(Camera::Status status, bool force);    
	  void _startAcq();
    inline bool IS_POWER_OF_2(long x)
    {
        return (x > 0) && ((x & (x - 1)) == 0);
    }
    //////////////////////////////
    // -- dhyana6060 specific members
    //////////////////////////////

    class AcqThread;

    AcqThread *         m_acq_thread;
    TrigMode            m_trigger_mode;
    double              m_exp_time;
    double              m_lat_time;
    ImageType           m_image_type;
    int                 m_nb_frames; // nos of frames to acquire
    bool                m_thread_running;
    bool                m_wait_flag;
    bool                m_quit;
    int                 m_acq_frame_nb; // nos of frames acquired
    mutable             Cond m_cond;
    long                m_depth;
    Camera::Status      m_status;
    Bin                 m_bin;
    double              m_temperature_target;
    // Buffer control object
    SoftBufferCtrlObj   m_bufferCtrlObj;
	  CSoftTriggerTimer*	m_internal_trigger_timer;
    double              m_fps;
	  unsigned short 		m_timer_period_ms;
    
    //TUCAM stuff, use TUCAM notations !
    TucamTriggerMode    m_tucam_trigger_mode;
    TucamTriggerEdge    m_tucam_trigger_edge_mode;
    TUCAM_TRGOUT_ATTR m_tgroutAttr1;
    TUCAM_TRGOUT_ATTR m_tgroutAttr2;
    TUCAM_TRGOUT_ATTR m_tgroutAttr3;

} ;

/*******************************************************************
 * \class AcqThread
 * \brief Thread of acquisition
 *******************************************************************/
class Camera::AcqThread : public Thread
{
    DEB_CLASS_NAMESPC(DebModCamera, "Camera", "AcqThread");
public:
    AcqThread(Camera &aCam);
    virtual ~AcqThread();

protected:
    virtual void threadFunction();

private:
    Camera& m_cam;
} ;

} // namespace Dhyana6060
} // namespace lima

#endif /* DHYANA6060CAMERA_H_ */
