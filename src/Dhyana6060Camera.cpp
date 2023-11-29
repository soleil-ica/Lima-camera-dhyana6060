//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2014
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

#include <sstream>
#include <iostream>
#include <string>
#include <math.h>
#include <climits>
#include <iomanip>
#include <signal.h>
#include <unistd.h>
#include <errno.h>
#include "lima/Exceptions.h"
#include "lima/Debug.h"
#include "lima/MiscUtils.h"
#include "Dhyana6060Timer.h"
#include "Dhyana6060Camera.h"

using namespace lima;
using namespace lima::Dhyana6060;
using namespace std;

//---------------------------
// @brief  Ctor
//---------------------------
Camera::Camera(unsigned short timer_period_ms):
m_depth(16),
m_trigger_mode(IntTrig),
m_status(Ready),
m_acq_frame_nb(0),
m_temperature_target(0),
m_timer_period_ms(timer_period_ms),
m_fps(0.0),
m_tucam_trigger_mode(kTriggerStandard),
m_tucam_trigger_edge_mode(kEdgeRising)
{

	DEB_CONSTRUCTOR();
	//Init TUCAM	
	init();
	//create the acquisition thread
	DEB_TRACE() << "Create the acquisition thread";
	m_acq_thread = new AcqThread(*this);
	DEB_TRACE() <<"Create the Internal Trigger Timer";
	m_internal_trigger_timer = new CSoftTriggerTimer(m_timer_period_ms, *this);
	m_acq_thread->start();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
Camera::~Camera()
{
	DEB_DESTRUCTOR();
	// Close camera
	DEB_TRACE() << "Close TUCAM API ...";
	TUCAM_Dev_Close(m_opCam.hIdxTUCam);
	// Uninitialize SDK API environment
	DEB_TRACE() << "Uninitialize TUCAM API ...";
	TUCAM_Api_Uninit();
	//delete the acquisition thread
	DEB_TRACE() << "Delete the acquisition thread";
	delete m_acq_thread;
	//delete the Internal Trigger Timer
	DEB_TRACE() << "Delete the Internal Trigger Timer";
	delete m_internal_trigger_timer;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::init()
{
	DEB_MEMBER_FUNCT();

	DEB_TRACE() << "Initialize TUCAM API ...";
	m_itApi.pstrConfigPath = NULL;//Camera parameters input saving path is not defined
	m_itApi.uiCamCount = 0;

	if(TUCAMRET_SUCCESS != TUCAM_Api_Init(&m_itApi))
	{
		// Initializing SDK API environment failed
		THROW_HW_ERROR(Error) << "Unable to initialize TUCAM_Api !";
	}

	if(0 == m_itApi.uiCamCount)
	{
		// No camera
		THROW_HW_ERROR(Error) << "Unable to locate the camera !";
	}

	DEB_TRACE() << "Open TUCAM API ...";
	m_opCam.hIdxTUCam = NULL;
	m_opCam.uiIdxOpen = 0;	
	if(TUCAMRET_SUCCESS != TUCAM_Dev_Open(&m_opCam))
	{
		// Failed to open camera
		THROW_HW_ERROR(Error) << "Unable to Open the camera !";
	}

	if(NULL == m_opCam.hIdxTUCam)
	{
		THROW_HW_ERROR(Error) << "Unable to open the camera !";
	}
	
	//initialize TUCAM Event used when Waiting for Frame
	m_hThdEvent = NULL;

	DEB_TRACE() << "Initialization ended ...";

}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::reset()
{
	DEB_MEMBER_FUNCT();
	stopAcq();	
	//@BEGIN : other stuff on Driver/API
	//...
	//@END
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::prepareAcq()
{
	DEB_MEMBER_FUNCT();
	AutoMutex lock(m_cond.mutex());
	Timestamp t0 = Timestamp::now();

	//@BEGIN : Ensure that Acquisition is Started before return ...
	DEB_TRACE() << "prepareAcq ...";
	DEB_TRACE() << "Ensure that Acquisition is Started";
	setStatus(Camera::Exposure, false);
	if(NULL == m_hThdEvent)
	{
		m_frame.pBuffer = NULL;
		m_frame.ucFormatGet = TUFRM_FMT_RAW;
		m_frame.uiRsdSize = 1;// how many frames do you want

		// Alloc buffer after set resolution or set ROI attribute
		DEB_TRACE() << "TUCAM_Buf_Alloc";
		TUCAM_Buf_Alloc(m_opCam.hIdxTUCam, &m_frame);

		DEB_TRACE() << "TUCAM_Cap_Start";
		TUCAM_Cap_Start(m_opCam.hIdxTUCam, TUCCM_SEQUENCE);
		TUCAM_ELEMENT node;
		node.pName = "AcquisitionTrigMode";
		if (TUCAMRET_SUCCESS == TUCAM_GenICam_ElementAttr(m_opCam.hIdxTUCam, &node, node.pName))
		{
			if(m_trigger_mode == IntTrig)
			{
				// Start capture in software trigger
				node.nVal = 2;  //0-FreeRunning / 1-Standard / 2-Software / 3-GPS
				TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
			}
			if(m_trigger_mode == IntTrigMult)
			{
				// Start capture in software trigger
				node.nVal = 2;  //0-FreeRunning / 1-Standard / 2-Software / 3-GPS
				TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
			}		
			else if(m_trigger_mode == ExtTrigMult)
			{
				// Start capture in external trigger STANDARD (EXPOSURE SOFT)
				node.nVal = 1;  //0-FreeRunning / 1-Standard / 2-Software / 3-GPS
				TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
			}
			else if(m_trigger_mode == ExtGate)
			{
				// Start capture in external trigger STANDARD (EXPOSURE WIDTH)
				node.nVal = 1;  //0-FreeRunning / 1-Standard / 2-Software / 3-GPS
				TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
			}
		}
		
		////DEB_TRACE() << "TUCAM CreateEvent";
		m_hThdEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	}
	
	//@BEGIN : trigger the acquisition
	if(m_trigger_mode == IntTrig)	
	{
		DEB_TRACE() <<"Start Internal Trigger Timer (Single)";
		m_internal_trigger_timer->disable_oneshot_mode();
		m_internal_trigger_timer->start();
	}

	
	//@END
	if(m_trigger_mode == IntTrigMult)
	{
	  _startAcq();
    }
	
	Timestamp t1 = Timestamp::now();
	double delta_time = t1 - t0;
	DEB_TRACE() << "prepareAcq : elapsed time = " << (int) (delta_time * 1000) << " (ms)";
	//@END
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::startAcq()
{
	DEB_MEMBER_FUNCT();
	AutoMutex lock(m_cond.mutex());
	
	Timestamp t0 = Timestamp::now();

	DEB_TRACE() << "startAcq ...";

	StdBufferCbMgr& buffer_mgr = m_bufferCtrlObj.getBuffer();
	buffer_mgr.setStartTimestamp(Timestamp::now());
	
	//@BEGIN : trigger the acquisition
	if(m_trigger_mode == IntTrigMult)	
	{
		DEB_TRACE() <<"Start Internal Trigger Timer (Multi)";
		m_internal_trigger_timer->enable_oneshot_mode();
		m_internal_trigger_timer->start();
		return;
	}
	//@END
	m_acq_frame_nb = 0;
	m_fps = 0.0;	
	DEB_TRACE() << "Ensure that Acquisition is Started  & wait thread to be started";
	setStatus(Camera::Exposure, false);			
	//Start acquisition thread & wait 
	{
		m_wait_flag = false;
		m_quit = false;
		m_cond.broadcast();
		m_cond.wait();
	}
	
	
	Timestamp t1 = Timestamp::now();
	double delta_time = t1 - t0;
	DEB_TRACE() << "startAcq : elapsed time = " << (int) (delta_time * 1000) << " (ms)";

}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::_startAcq()
{
  DEB_MEMBER_FUNCT();
  m_acq_frame_nb = 0;
  m_fps = 0.0;	
  //Start acqusition thread
  AutoMutex aLock(m_cond.mutex());
  m_wait_flag = false;
  m_cond.broadcast();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::stopAcq()
{
	DEB_MEMBER_FUNCT();
	AutoMutex aLock(m_cond.mutex());
	DEB_TRACE() << "stopAcq ...";
	// Don't do anything if acquisition is idle.
	if(m_thread_running == true)
	{
		m_wait_flag = true;
		m_cond.broadcast();		
	}

	//@BEGIN : Ensure that Acquisition is Stopped before return ...			
	Timestamp t0 = Timestamp::now();
	if(NULL != m_hThdEvent)
	{
		DEB_TRACE() << "TUCAM_Buf_AbortWait";
		TUCAM_Buf_AbortWait(m_opCam.hIdxTUCam);
		WaitForSingleObject(m_hThdEvent, INFINITE);
		CloseHandle(m_hThdEvent);
		m_hThdEvent = NULL;
		// Stop capture   
		DEB_TRACE() << "TUCAM_Cap_Stop";
		TUCAM_Cap_Stop(m_opCam.hIdxTUCam);
		// Release alloc buffer after stop capture
		DEB_TRACE() << "TUCAM_Buf_Release";
		TUCAM_Buf_Release(m_opCam.hIdxTUCam);
	}
	//@END	
	
	//@BEGIN : trigger the acquisition
	if(m_trigger_mode == IntTrig)	
	{
		DEB_TRACE() <<"Stop Internal Trigger Timer (Single)";
		m_internal_trigger_timer->stop();
	}
	//@END
	
	//@BEGIN : trigger the acquisition
	if(m_trigger_mode == IntTrigMult)	
	{
		DEB_TRACE() <<"Stop Internal Trigger Timer (Multi)";
		m_internal_trigger_timer->stop();
	}
	//@END
	
	//@BEGIN
	//now detector is ready
	DEB_TRACE() << "Ensure that Acquisition is Stopped";
	setStatus(Camera::Ready, false);
	//@END	
	
	Timestamp t1 = Timestamp::now();
	double delta_time = t1 - t0;
	DEB_TRACE() << "stopAcq : elapsed time = " << (int) (delta_time * 1000) << " (ms)";		

}

//-----------------------------------------------------
// @brief set the new camera status
//-----------------------------------------------------
void Camera::setStatus(Camera::Status status, bool force)
{
	DEB_MEMBER_FUNCT();
	if(force || m_status != Camera::Fault)
	{
		m_status = status;
	}
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getStatus(Camera::Status& status)
{
	DEB_MEMBER_FUNCT();
	AutoMutex aLock(m_cond.mutex());
	if(m_trigger_mode == IntTrigMult)
	{
		m_status = Camera::Ready;
	}
		
	status = m_status;

	DEB_RETURN() << DEB_VAR1(status);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
bool Camera::readFrame(void *bptr, int& frame_nb)
{
	DEB_MEMBER_FUNCT();
	Timestamp t0 = Timestamp::now();

	//@BEGIN : Get frame from Driver/API & copy it into bptr already allocated 
	DEB_TRACE() << "Copy Buffer image into Lima Frame Ptr";
	memcpy((unsigned short *) bptr, (unsigned short *) (m_frame.pBuffer + m_frame.usOffset), m_frame.uiImgSize);//we need a nb of BYTES .		
	frame_nb = m_frame.uiIndex;
	//@END	

	Timestamp t1 = Timestamp::now();
	double delta_time = t1 - t0;
	DEB_TRACE() << "readFrame : elapsed time = " << (int) (delta_time * 1000) << " (ms)";
	return false;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::AcqThread::threadFunction()
{
	DEB_MEMBER_FUNCT();
	AutoMutex aLock(m_cam.m_cond.mutex());
	StdBufferCbMgr& buffer_mgr = m_cam.m_bufferCtrlObj.getBuffer();

	while(!m_cam.m_quit)
	{
		while(m_cam.m_wait_flag && !m_cam.m_quit)
		{
			DEB_TRACE() << "Wait for start acquisition ...";
			m_cam.m_thread_running = false;
			m_cam.m_cond.broadcast();
			m_cam.m_cond.wait();
		}

		//if quit is requested (requested only by destructor)
		if(m_cam.m_quit)
			return;

		DEB_TRACE() << "Running ...";
		m_cam.m_thread_running = true;
		m_cam.m_cond.broadcast();
		aLock.unlock();		

		Timestamp t0_capture = Timestamp::now();
		Timestamp t0_fps, t1_fps, delta_fps;

		//@BEGIN 
		DEB_TRACE() << "Capture all frames ...";
		bool continueFlag = true;
		t0_fps = Timestamp::now();
		while(continueFlag && (!m_cam.m_nb_frames || m_cam.m_acq_frame_nb < m_cam.m_nb_frames))
		{
			// Check first if acq. has been stopped
			if(m_cam.m_wait_flag)
			{
				DEB_TRACE() << "AcqThread has been stopped from user";
				continueFlag = false;
				continue;
			}
			
			//wait frame from TUCAM API ...
			if(m_cam.m_acq_frame_nb == 0)//display TRACE only once ...
			{				
				DEB_TRACE() << "TUCAM_Buf_WaitForFrame ...";
			}
			
			if(TUCAMRET_SUCCESS == TUCAM_Buf_WaitForFrame(m_cam.m_opCam.hIdxTUCam, &m_cam.m_frame))
			{
				//The based information
				/*DEB_TRACE() << "m_cam.m_frame.szSignature = "	<< m_cam.m_frame.szSignature;		// [out]Copyright+Version: TU+1.0 ['T', 'U', '1', '\0']		
				DEB_TRACE() << "m_cam.m_frame.usHeader = "	<< m_cam.m_frame.usHeader;			// [out] The frame header size
				DEB_TRACE() << "m_cam.m_frame.usOffset = "	<< m_cam.m_frame.usOffset;			// [out] The frame data offset
				DEB_TRACE() << "m_cam.m_frame.usWidth = "		<< m_cam.m_frame.usWidth;						// [out] The frame width
				DEB_TRACE() << "m_cam.m_frame.usHeight = "	<< m_cam.m_frame.usHeight;						// [out] The frame height
				DEB_TRACE() << "m_cam.m_frame.uiWidthStep = "	<< m_cam.m_frame.uiWidthStep;		// [out] The frame width step
				DEB_TRACE() << "m_cam.m_frame.ucDepth = "		<< m_cam.m_frame.ucDepth;			// [out] The frame data depth 
				DEB_TRACE() << "m_cam.m_frame.ucFormat = "	<< m_cam.m_frame.ucFormat;			// [out] The frame data format                  
				DEB_TRACE() << "m_cam.m_frame.ucChannels = "	<< m_cam.m_frame.ucChannels;			// [out] The frame data channels
				DEB_TRACE() << "m_cam.m_frame.ucElemBytes = "	<< m_cam.m_frame.ucElemBytes;		// [out] The frame data bytes per element
				DEB_TRACE() << "m_cam.m_frame.ucFormatGet = "	<< m_cam.m_frame.ucFormatGet;		// [in]  Which frame data format do you want    see TUFRM_FORMATS
				DEB_TRACE() << "m_cam.m_frame.uiIndex = "		<< m_cam.m_frame.uiIndex;						// [in/out] The frame index number
				DEB_TRACE() << "m_cam.m_frame.uiImgSize = "	<< m_cam.m_frame.uiImgSize;						// [out] The frame size
				DEB_TRACE() << "m_cam.m_frame.uiRsdSize = "	<< m_cam.m_frame.uiRsdSize;						// [in]  The frame reserved size    (how many frames do you want)
				*/
			
				// Grabbing was successful, process image
				m_cam.setStatus(Camera::Readout, false);

				//Prepare Lima Frame Ptr 
				void* bptr = buffer_mgr.getFrameBufferPtr(m_cam.m_acq_frame_nb);

				//Copy Frame into Lima Frame Ptr
				int frame_nb = 0;
				m_cam.readFrame(bptr, frame_nb);
		
				//Push the image buffer through Lima 
				Timestamp t0 = Timestamp::now();
				DEB_TRACE() << "Declare a Lima new Frame Ready (" << m_cam.m_acq_frame_nb << ")";
				HwFrameInfoType frame_info;
				frame_info.acq_frame_nb = m_cam.m_acq_frame_nb;
				continueFlag = buffer_mgr.newFrameReady(frame_info);
				m_cam.m_acq_frame_nb++;
				
				Timestamp t1 = Timestamp::now();
				double delta_time = t1 - t0;			

				//wait latency after each frame , except for the last image 
				if((!m_cam.m_nb_frames) || (m_cam.m_acq_frame_nb < m_cam.m_nb_frames) && (m_cam.m_lat_time))
				{
					DEB_TRACE() << "Wait latency time : " << m_cam.m_lat_time * 1000 << " (ms) ...";
					usleep((DWORD) (m_cam.m_lat_time * 1000000));
				}		
				DEB_TRACE() << "newFrameReady+latency : elapsed time = " << (int) (delta_time * 1000) << " (ms)";						
			}
			else
			{
				DEB_TRACE() << "Unable to get the frame from the camera !";
			}


			t1_fps = Timestamp::now();
			delta_fps = t1_fps - t0_fps;
			if (delta_fps > 0)
			{
				m_cam.m_fps = m_cam.m_acq_frame_nb / delta_fps;
			}
		}

		//
		////DEB_TRACE() << "TUCAM SetEvent";
		SetEvent(m_cam.m_hThdEvent);
		//@END
		
		//stopAcq only if this is not already done		
		DEB_TRACE() << "stopAcq only if this is not already done";
		if(!m_cam.m_wait_flag)
		{
			////DEB_TRACE() << "stopAcq";
			m_cam.stopAcq();
		}

		//now detector is ready
		m_cam.setStatus(Camera::Ready, false);
		DEB_TRACE() << "AcqThread is no more running";		
		
		Timestamp t1_capture = Timestamp::now();
		double delta_time_capture = t1_capture - t0_capture;

		DEB_TRACE() << "Capture all frames elapsed time = " << (int) (delta_time_capture * 1000) << " (ms)";				

		aLock.lock();
		m_cam.m_thread_running = false;
		m_cam.m_wait_flag = true;
	}
}

//-----------------------------------------------------
//
//-----------------------------------------------------
Camera::AcqThread::AcqThread(Camera& cam):
m_cam(cam)
{
	AutoMutex aLock(m_cam.m_cond.mutex());
	m_cam.m_wait_flag = true;
	m_cam.m_quit = false;
	aLock.unlock();
	pthread_attr_setscope(&m_thread_attr, PTHREAD_SCOPE_PROCESS);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
Camera::AcqThread::~AcqThread()
{
	AutoMutex aLock(m_cam.m_cond.mutex());
	m_cam.m_wait_flag = true;
	m_cam.m_quit = true;
	m_cam.m_cond.broadcast();
	aLock.unlock();
	join();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getImageType(ImageType& type)
{
	DEB_MEMBER_FUNCT();
	//@BEGIN : Fix the image type (pixel depth) into Driver/API		
	switch(m_depth)
	{
		case 16: type = Bpp16;
			break;
		default:
			THROW_HW_ERROR(Error) << "This pixel format of the camera is not managed, only 16 bits cameras are already managed!";
			break;
	}
	//@END	
	return;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setImageType(ImageType type)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "setImageType - " << DEB_VAR1(type);
	//@BEGIN : Fix the image type (pixel depth) into Driver/API	
	switch(type)
	{
		case Bpp16:
			m_depth = 16;
			break;
		default:
			THROW_HW_ERROR(Error) << "This pixel format of the camera is not managed, only 16 bits cameras are already managed!";
			break;
	}
	//@END
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getDetectorType(std::string& type)
{
	DEB_MEMBER_FUNCT();
	//@BEGIN : Get Detector type from Driver/API
	type = "Tucsen - Dhyana6060";
	//@END	

}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getDetectorModel(std::string& model)
{
	DEB_MEMBER_FUNCT();
	TUCAM_ELEMENT node; // Property node
	int err = TUCAM_GenICam_ElementAttr(m_opCam.hIdxTUCam, &node, "DeviceModelName");
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to Read DeviceModelName from the camera ! Error: " << err << " ";
	}
	model = node.pTransfer;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getDetectorImageSize(Size& size)
{
	DEB_MEMBER_FUNCT();
	TUCAM_ELEMENT node; // Property node
	int err = TUCAM_GenICam_ElementAttr(m_opCam.hIdxTUCam, &node, "SensorWidth");
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to Read SensorWidth from the camera ! Error: " << err << " ";
	}
	int x = node.nVal;
	err = TUCAM_GenICam_ElementAttr(m_opCam.hIdxTUCam, &node, "SensorHeight");
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to Read SensorHeight from the camera ! Error: " << err << " ";
	}
	int y = node.nVal;
	size = Size(x, y);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getPixelSize(double& sizex, double& sizey)
{
	DEB_MEMBER_FUNCT();
	TUCAM_ELEMENT node; // Property node
	int err = TUCAM_GenICam_ElementAttr(m_opCam.hIdxTUCam, &node, "SensorPixelSize");
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to Read SensorPixelSize from the camera ! Error: " << err << " ";
	}
	sizex = node.nVal;
	sizey = node.nVal;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
HwBufferCtrlObj* Camera::getBufferCtrlObj()
{
	return &m_bufferCtrlObj;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
bool Camera::checkTrigMode(TrigMode mode)
{
	DEB_MEMBER_FUNCT();
	bool valid_mode;
	//@BEGIN
	switch(mode)
	{
		case IntTrig:
		case IntTrigMult:
		case ExtTrigMult:
		case ExtGate:
			valid_mode = true;
			break;
		case ExtTrigReadout:
		case ExtTrigSingle:
		default:
			valid_mode = false;
			break;
	}
	//@END
	return valid_mode;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setTrigMode(TrigMode mode)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "setTrigMode() " << DEB_VAR1(mode);
	DEB_PARAM() << DEB_VAR1(mode);
	//@BEGIN

	TUCAMRET err;
	TUCAM_ELEMENT node; // Property node
	node.pName = "AcquisitionTrigMode";
	node.nVal = 1;  //0-FreeRunning / 1-Standard / 2-Software / 3-GPS
	TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);

	switch(mode)
	{
		case IntTrig:

			node.pName = "TrigExpType";
			node.nVal = TUCTE_EXPTM;
			err = TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
			if(TUCAMRET_SUCCESS != err)
			{
				THROW_HW_ERROR(Error) << "Unable to set TrigInExpType target from the camera ! Error: " << err << " ";
			}

			node.pName = "AcquisitionTrigMode";
			node.nVal = 2;  //0-FreeRunning / 1-Standard / 2-Software / 3-GPS
			TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
			DEB_TRACE() << "TUCAM_Cap_SetTrigger : TUCCM_TRIGGER_SOFTWARE (EXPOSURE SOFTWARE)";
			break;
		case IntTrigMult:
			node.pName = "TrigExpType";
			node.nVal = TUCTE_EXPTM;
			err = TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
			if(TUCAMRET_SUCCESS != err)
			{
				THROW_HW_ERROR(Error) << "Unable to set TrigInExpType target from the camera ! Error: " << err << " ";
			}

			node.pName = "AcquisitionTrigMode";
			node.nVal = 2;  //0-FreeRunning / 1-Standard / 2-Software / 3-GPS
			TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
			DEB_TRACE() << "TUCAM_Cap_SetTrigger : TUCCM_TRIGGER_SOFTWARE (EXPOSURE SOFTWARE) (MULTI)";
			break;			
		case ExtTrigMult :
			//already in standard mode


			node.pName = "TrigExpType";
			node.nVal = TUCTE_EXPTM;
			err = TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
			if(TUCAMRET_SUCCESS != err)
			{
				THROW_HW_ERROR(Error) << "Unable to set TrigInExpType target from the camera ! Error: " << err << " ";
			}
			DEB_TRACE() << "TUCAM_Cap_SetTrigger : TUCCM_TRIGGER_STANDARD (EXPOSURE SOFTWARE: TrigTimed)";
			break;
		case ExtGate:
			//already in standard mode

			node.pName = "TrigExpType";
			node.nVal = TUCTE_WIDTH;
			err = TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
			if(TUCAMRET_SUCCESS != err)
			{
				THROW_HW_ERROR(Error) << "Unable to set TrigInExpType target from the camera ! Error: " << err << " ";
			}
			DEB_TRACE() << "TUCAM_GenICam_SetElementValue : TUCCM_TRIGGER_STANDARD (EXPOSURE TRIGGER WIDTH: TrigWidth)";
			break;			
		case ExtTrigSingle :
		case ExtTrigReadout:
		default:
			THROW_HW_ERROR(NotSupported) << DEB_VAR1(mode);
	}
	m_trigger_mode = mode;
	//@END
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getTrigMode(TrigMode& mode)
{
	DEB_MEMBER_FUNCT();
	mode = m_trigger_mode;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getTriggerMode(TucamTriggerMode &mode)
{
	DEB_MEMBER_FUNCT();
	mode = m_tucam_trigger_mode;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setTriggerMode(TucamTriggerMode mode)
{
	DEB_MEMBER_FUNCT();
	m_tucam_trigger_mode = mode;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getTriggerEdge(TucamTriggerEdge &edge)
{
	DEB_MEMBER_FUNCT();
	edge = m_tucam_trigger_edge_mode;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setTriggerEdge(TucamTriggerEdge edge)
{
	DEB_MEMBER_FUNCT();
	m_tucam_trigger_edge_mode = edge;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getExpTime(double& exp_time)
{
	DEB_MEMBER_FUNCT();
	TUCAM_ELEMENT node; // Property node
	int err = TUCAM_GenICam_ElementAttr(m_opCam.hIdxTUCam, &node, "AcquisitionExpTime");
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to Read AcquisitionExpTime from the camera ! Error: " << err << " ";
	}
	exp_time = node.nVal / 1000000; //TUCAM use (us), but lima use (second) as unit
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setExpTime(double exp_time)
{
	DEB_MEMBER_FUNCT();
	TUCAMRET err;
	TUCAM_ELEMENT node; // Property node
	node.nVal = exp_time * 1000000; //TUCAM use (us), but lima use (second) as unit 
	node.pName = "AcquisitionExpTime";
	err = TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to set AcquisitionExpTime from the camera ! Error: " << err << " ";
	}
}


//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setLatTime(double lat_time)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "setLatTime() " << DEB_VAR1(lat_time);
	m_lat_time = lat_time;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getLatTime(double& lat_time)
{
	DEB_MEMBER_FUNCT();
	//@BEGIN
	//@END
	m_lat_time = lat_time;
	DEB_RETURN() << DEB_VAR1(lat_time);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getExposureTimeRange(double& min_expo, double& max_expo) const
{
	DEB_MEMBER_FUNCT();
	//@BEGIN
	min_expo = 0.;
	max_expo = 10;//10s
	//@END
	DEB_RETURN() << DEB_VAR2(min_expo, max_expo);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getLatTimeRange(double& min_lat, double& max_lat) const
{
	DEB_MEMBER_FUNCT();
	//@BEGIN
	// --- no info on min latency
	min_lat = 0.;
	// --- do not know how to get the max_lat, fix it as the max exposure time
	max_lat = 10;//10s
	//@END
	DEB_RETURN() << DEB_VAR2(min_lat, max_lat);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setNbFrames(int nb_frames)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "setNbFrames() " << DEB_VAR1(nb_frames);
	//@BEGIN
	if(nb_frames < 0)
	{
		THROW_HW_ERROR(Error) << "Number of frames to acquire has not been set";
	}
	//@END
	m_nb_frames = nb_frames;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getNbFrames(int& nb_frames)
{
	DEB_MEMBER_FUNCT();
	DEB_RETURN() << DEB_VAR1(m_nb_frames);
	nb_frames = m_nb_frames;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
int Camera::getNbHwAcquiredFrames()
{
	DEB_MEMBER_FUNCT();
	return m_acq_frame_nb;
}

//-----------------------------------------------------
// @brief range the binning to the maximum allowed
//-----------------------------------------------------
void Camera::checkBin(Bin &hw_bin)
{
	DEB_MEMBER_FUNCT();
	//@BEGIN : check available values of binning H/V
	int x = hw_bin.getX();
	int y = hw_bin.getY();
	if(x != 1 || y != 1)
	{
		DEB_ERROR() << "Binning values not supported";
		THROW_HW_ERROR(Error) << "Binning values not supported = " << DEB_VAR1(hw_bin);
	}
	//@END

	hw_bin = Bin(x, y);
	DEB_RETURN() << DEB_VAR1(hw_bin);
}
//-----------------------------------------------------
// @brief set the new binning mode
//-----------------------------------------------------
void Camera::setBin(const Bin &set_bin)
{
	DEB_MEMBER_FUNCT();
	m_bin = set_bin;

	DEB_RETURN() << DEB_VAR1(set_bin);
}

//-----------------------------------------------------
// @brief return the current binning mode
//-----------------------------------------------------
void Camera::getBin(Bin &hw_bin)
{
	DEB_MEMBER_FUNCT();
	//@BEGIN : get binning from Driver/API
	int bin_x = 1;
	int bin_y = 1;
	//@END
	Bin tmp_bin(bin_x, bin_y);

	hw_bin = tmp_bin;
	m_bin = tmp_bin;

	DEB_RETURN() << DEB_VAR1(hw_bin);
}

//-----------------------------------------------------
//! Camera::checkRoi()
//-----------------------------------------------------
void Camera::checkRoi(const Roi& set_roi, Roi& hw_roi)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "checkRoi";
	DEB_PARAM() << DEB_VAR1(set_roi);
	//@BEGIN : check available values of Roi
	if(set_roi.isActive())
	{
		if ((set_roi.getSize().getWidth() % 16 != 0) ||
			(set_roi.getSize().getHeight() % 4 != 0) ||
			(set_roi.getTopLeft().x % 16 != 0) ||
			(set_roi.getTopLeft().y % 4 != 0))
		{
			THROW_HW_ERROR(Error) << "Roi coordinates (x, y, width, height) must respect some constraints:\n"
								  << " - x must be a multiple of 16\n"
								  << " - y must be a multiple of 4\n"
								  << " - width must be a multiple of 16\n"
								  << " - height must be a multiple of 4\n";
		}
		hw_roi = set_roi;

	}
	else
	{
		hw_roi = set_roi;
	}
	//@END


	DEB_RETURN() << DEB_VAR1(hw_roi);
}

//---------------------------------------------------------------------------------------
//! Camera::getRoi()
//---------------------------------------------------------------------------------------
void Camera::getRoi(Roi& hw_roi)
{
	DEB_MEMBER_FUNCT();
	//@BEGIN : get Roi from the Driver/API
	TUCAM_ELEMENT node; // Property node

	// Get ROI Width
	int err = TUCAM_GenICam_ElementAttr(m_opCam.hIdxTUCam, &node, "Width");
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to Read ROIWidth from the camera ! Error: " << err << " ";
	}
	double width = node.nVal;

	// Get ROI Height
	err = TUCAM_GenICam_ElementAttr(m_opCam.hIdxTUCam, &node, "Height");
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to Read ROIHeight from the camera ! Error: " << err << " ";
	}
	double height = node.nVal;

	// Get ROI Offset x
	err = TUCAM_GenICam_ElementAttr(m_opCam.hIdxTUCam, &node, "OffsetX");
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to Read ROIOffsetX from the camera ! Error: " << err << " ";
	}
	double x_offset = node.nVal;

	// Get ROI Offset y
	err = TUCAM_GenICam_ElementAttr(m_opCam.hIdxTUCam, &node, "OffsetY");
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to Read ROIOffsetY from the camera ! Error: " << err << " ";
	}
	double y_offset = node.nVal;
	hw_roi = Roi(x_offset,
				y_offset,
				width,
				height);
	//@END
	DEB_RETURN() << DEB_VAR1(hw_roi);
}

//---------------------------------------------------------------------------------------
//! Camera::setRoi()
//---------------------------------------------------------------------------------------
void Camera::setRoi(const Roi& set_roi)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "setRoi";
	DEB_PARAM() << DEB_VAR1(set_roi);
	//@BEGIN : set Roi from the Driver/API	
	//set Roi to Driver/API
	Size size;
	getDetectorImageSize(size);
	if(!set_roi.isActive())
	{
		DEB_TRACE() << "Roi is not Enabled : so set full frame";


		TUCAMRET err;
		TUCAM_ELEMENT node; // Property node
		node.nVal = 0;
		node.pName = "OffsetX";
		err = TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
		if(TUCAMRET_SUCCESS != err)
		{
			THROW_HW_ERROR(Error) << "Unable to set OffsetX from the camera ! Error: " << err << " ";
		}
		node.pName = "OffsetY";
		err = TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
		if(TUCAMRET_SUCCESS != err)
		{
			THROW_HW_ERROR(Error) << "Unable to set OffsetY from the camera ! Error: " << err << " ";
		}
		node.nVal = size.getWidth();
		node.pName = "Width";
		err = TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
		if(TUCAMRET_SUCCESS != err)
		{
			THROW_HW_ERROR(Error) << "Unable to set SensorWidth from the camera ! Error: " << err << " ";
		}
		node.nVal = size.getHeight();
		node.pName = "Height";
		err = TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
		if(TUCAMRET_SUCCESS != err)
		{
			THROW_HW_ERROR(Error) << "Unable to set SensorHeight from the camera ! Error: " << err << " ";
		}
	}
	else
	{
		DEB_TRACE() << "Roi is Enabled";

		TUCAMRET err;
		TUCAM_ELEMENT node; // Property node
		node.nVal = set_roi.getTopLeft().x;
		node.pName = "OffsetX";
		err = TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
		if(TUCAMRET_SUCCESS != err)
		{
			THROW_HW_ERROR(Error) << "Unable to set ROIOffsetX from the camera ! Error: " << err << " ";
		}
		node.nVal = size.getHeight() - set_roi.getTopLeft().y - set_roi.getSize().getHeight();
		node.pName = "OffsetY";
		err = TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
		if(TUCAMRET_SUCCESS != err)
		{
			THROW_HW_ERROR(Error) << "Unable to set ROIOffsetY from the camera ! Error: " << err << " ";
		}
		node.nVal = set_roi.getSize().getWidth();
		node.pName = "Width";
		err = TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
		if(TUCAMRET_SUCCESS != err)
		{
			THROW_HW_ERROR(Error) << "Unable to set ROIWidth from the camera ! Error: " << err << " ";
		}
		node.nVal = set_roi.getSize().getHeight();
		node.pName = "Height";
		err = TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
		if(TUCAMRET_SUCCESS != err)
		{
			THROW_HW_ERROR(Error) << "Unable to set ROIHeight from the camera ! Error: " << err << " ";
		}
	}
	//getRoi(set_roi);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
bool Camera::isAcqRunning() const
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "isAcqRunning - " << DEB_VAR1(m_thread_running) << "---------------------------";
	return m_thread_running;
}

///////////////////////////////////////////////////////
// Dhyana specific stuff now
///////////////////////////////////////////////////////

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::getCameraTemperature(double& temp)
{
	DEB_MEMBER_FUNCT();

	TUCAM_ELEMENT node; // Property node
	int err = TUCAM_GenICam_ElementAttr(m_opCam.hIdxTUCam, &node, "DeviceTemperature");
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to Read CameraTemperature from the camera ! Error: " << err << " ";
	}
	temp = node.dbVal;
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::setSensorTemperatureTarget(double temp)
{
	DEB_MEMBER_FUNCT();

	TUCAMRET err;
	TUCAM_ELEMENT node; // Property node
	node.dbVal = temp;
	node.pName = "SetSensorTemperature";
	err = TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to set sensor temperature target from the camera ! Error: " << err << " ";
	}
	m_temperature_target = temp;
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::getSensorTemperatureTarget(double& temp)
{
	DEB_MEMBER_FUNCT();
	TUCAM_ELEMENT node; // Property node
	int err = TUCAM_GenICam_ElementAttr(m_opCam.hIdxTUCam, &node, "SetSensorTemperature");
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to Read SensorTemperature from the camera ! Error: " << err << " ";
	}
	temp = node.dbVal;
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::getSensorTemperature(double& temp)
{
	DEB_MEMBER_FUNCT();

	TUCAM_ELEMENT node; // Property node
	int err = TUCAM_GenICam_ElementAttr(m_opCam.hIdxTUCam, &node, "SensorTemperature");
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to Read SensorTemperature from the camera ! Error: " << err << " ";
	}
	temp = node.dbVal;
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::setSensorCoolingType(unsigned type)
{
	DEB_MEMBER_FUNCT();
	TUCAMRET err;
	TUCAM_ELEMENT node; // Property node
	node.nVal = type;
	node.pName = "SensorCoolType";
	err = TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to set SensorCoolingType from the camera ! Error: " << err << " ";
	}
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::getSensorCoolingType(unsigned& type)
{
	DEB_MEMBER_FUNCT();
	TUCAM_ELEMENT node; // Property node
	int err = TUCAM_GenICam_ElementAttr(m_opCam.hIdxTUCam, &node, "SensorCoolType");
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to Read SensorCoolingType from the camera ! Error: " << err << " ";
	}
	type = node.nVal;
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::setFanSpeed(unsigned speed)
{
	DEB_MEMBER_FUNCT();
	TUCAMRET err;
	TUCAM_ELEMENT node; // Property node
	node.dbVal = speed;
	node.pName = "FanSpeed";
	err = TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to set FanSpeed from the camera ! Error: " << err << " ";
	}
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::getFanSpeed(unsigned& speed)
{
	DEB_MEMBER_FUNCT();
	TUCAM_ELEMENT node; // Property node
	int err = TUCAM_GenICam_ElementAttr(m_opCam.hIdxTUCam, &node, "FanSpeed");
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to Read FanSpeed from the camera ! Error: " << err << " ";
	}
	speed = node.dbVal;
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::setFanType(unsigned type)
{
	DEB_MEMBER_FUNCT();
	TUCAMRET err;
	TUCAM_ELEMENT node; // Property node
	node.nVal = type;
	node.pName = "FanAuto";
	err = TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to set FanType from the camera ! Error: " << err << " ";
	}
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::getFanType(unsigned& type)
{
	DEB_MEMBER_FUNCT();
	TUCAM_ELEMENT node; // Property node
	int err = TUCAM_GenICam_ElementAttr(m_opCam.hIdxTUCam, &node, "FanAuto");
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to Read FanType from the camera ! Error: " << err << " ";
	}
	type = node.nVal;
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::setGlobalGain(unsigned gain)
{
	DEB_MEMBER_FUNCT();
	TUCAMRET err;
	TUCAM_ELEMENT node; // Property node
	node.nVal = gain;
	node.pName = "GainMode";
	err = TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to set GlobalGain from the camera ! Error: " << err << " ";
	}
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::getGlobalGain(unsigned& gain)
{
	DEB_MEMBER_FUNCT();
	TUCAM_ELEMENT node; // Property node
	int err = TUCAM_GenICam_ElementAttr(m_opCam.hIdxTUCam, &node, "GainMode");
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to Read GlobalGain from the camera ! Error: " << err << " ";
	}
	gain = node.nVal;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getTucamVersion(std::string& version)
{
	DEB_MEMBER_FUNCT();
	TUCAM_ELEMENT node; // Property node
	int err = TUCAM_GenICam_ElementAttr(m_opCam.hIdxTUCam, &node, "DeviceVersion");
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to Read DeviceVersion from the camera ! Error: " << err << " ";
	}
	version = node.pTransfer;
}

//-----------------------------------------------------------------------------
/// Get the frame rate
//-----------------------------------------------------------------------------
void Camera::getFPS(double& fps) ///< [out] last computed fps
{
    DEB_MEMBER_FUNCT();
	//TODO
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::setTrigInEdge(unsigned edge)
{
	DEB_MEMBER_FUNCT();

	TUCAMRET err;
	TUCAM_ELEMENT node; // Property node
	node.nVal = edge;
	node.pName = "TrigEdge";
	err = TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to set TrigInEdge target from the camera ! Error: " << err << " ";
	}
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::getTrigInEdge(unsigned& edge)
{
	DEB_MEMBER_FUNCT();
	TUCAM_ELEMENT node; // Property node
	int err = TUCAM_GenICam_ElementAttr(m_opCam.hIdxTUCam, &node, "TrigEdge");
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to Read TrigInEdge from the camera ! Error: " << err << " ";
	}
	edge = node.nVal;
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::setTrigInExpType(unsigned type)
{
	DEB_MEMBER_FUNCT();

	TUCAMRET err;
	TUCAM_ELEMENT node; // Property node
	node.nVal = type;
	node.pName = "TrigExpType";
	err = TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to set TrigInExpType target from the camera ! Error: " << err << " ";
	}
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::getTrigInExpType(unsigned& type)
{
	DEB_MEMBER_FUNCT();
	TUCAM_ELEMENT node; // Property node
	int err = TUCAM_GenICam_ElementAttr(m_opCam.hIdxTUCam, &node, "TrigExpType");
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to Read TrigInExpType from the camera ! Error: " << err << " ";
	}
	type = node.nVal;
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::setTrigInDelay(double delay)
{
	DEB_MEMBER_FUNCT();

	TUCAMRET err;
	TUCAM_ELEMENT node; // Property node
	node.nVal = delay;
	node.pName = "TrigDelay";
	err = TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to set TrigInDelay target from the camera ! Error: " << err << " ";
	}
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::getTrigInDelay(double& delay)
{
	DEB_MEMBER_FUNCT();
	TUCAM_ELEMENT node; // Property node
	int err = TUCAM_GenICam_ElementAttr(m_opCam.hIdxTUCam, &node, "TrigDelay");
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to Read TrigInDelay from the camera ! Error: " << err << " ";
	}
	delay = node.nVal;
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::setTrigOutputPort(unsigned port)
{
	DEB_MEMBER_FUNCT();

	TUCAMRET err;
	TUCAM_ELEMENT node; // Property node
	node.nVal = port;
	node.pName = "TrigOutputPort";
	err = TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to set TrigOutputPort target from the camera ! Error: " << err << " ";
	}
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::getTrigOutputPort(unsigned& port)
{
	DEB_MEMBER_FUNCT();
	TUCAM_ELEMENT node; // Property node
	int err = TUCAM_GenICam_ElementAttr(m_opCam.hIdxTUCam, &node, "TrigOutputPort");
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to Read TrigOutputPort from the camera ! Error: " << err << " ";
	}
	port = node.nVal;
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::setTrigOutputKind(unsigned kind)
{
	DEB_MEMBER_FUNCT();

	TUCAMRET err;
	TUCAM_ELEMENT node; // Property node
	node.nVal = kind;
	node.pName = "TrigOutputKind";
	err = TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to set TrigOutputKind target from the camera ! Error: " << err << " ";
	}
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::getTrigOutputKind(unsigned& kind)
{
	DEB_MEMBER_FUNCT();
	TUCAM_ELEMENT node; // Property node
	int err = TUCAM_GenICam_ElementAttr(m_opCam.hIdxTUCam, &node, "TrigOutputKind");
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to Read TrigOutputKind from the camera ! Error: " << err << " ";
	}
	kind = node.nVal;
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::setTrigOutputWidth(double width)
{
	DEB_MEMBER_FUNCT();

	TUCAMRET err;
	TUCAM_ELEMENT node; // Property node
	node.nVal = width;
	node.pName = "TrigOutputWidth";
	err = TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to set TrigOutputWidth target from the camera ! Error: " << err << " ";
	}
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::getTrigOutputWidth(double& width)
{
	DEB_MEMBER_FUNCT();
	TUCAM_ELEMENT node; // Property node
	int err = TUCAM_GenICam_ElementAttr(m_opCam.hIdxTUCam, &node, "TrigOutputWidth");
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to Read TrigOutputWidth from the camera ! Error: " << err << " ";
	}
	width = node.nVal;
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::setTrigOutputDelay(double delay)
{
	DEB_MEMBER_FUNCT();

	TUCAMRET err;
	TUCAM_ELEMENT node; // Property node
	node.nVal = delay;
	node.pName = "TrigOutputDelay";
	err = TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to set TrigOutputDelay target from the camera ! Error: " << err << " ";
	}
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::getTrigOutputDelay(double& delay)
{
	DEB_MEMBER_FUNCT();
	TUCAM_ELEMENT node; // Property node
	int err = TUCAM_GenICam_ElementAttr(m_opCam.hIdxTUCam, &node, "TrigOutputDelay");
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to Read TrigOutputDelay from the camera ! Error: " << err << " ";
	}
	delay = node.nVal;
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::setTrigOutputEdge(unsigned edge)
{
	DEB_MEMBER_FUNCT();

	TUCAMRET err;
	TUCAM_ELEMENT node; // Property node
	node.nVal = edge;
	node.pName = "TrigOutputEdge";
	err = TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to set TrigOutputEdge target from the camera ! Error: " << err << " ";
	}
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::getTrigOutputEdge(unsigned& edge)
{
	DEB_MEMBER_FUNCT();
	TUCAM_ELEMENT node; // Property node
	int err = TUCAM_GenICam_ElementAttr(m_opCam.hIdxTUCam, &node, "TrigOutputEdge");
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to Read TrigOutputEdge from the camera ! Error: " << err << " ";
	}
	edge = node.nVal;
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::setSensorCooling(unsigned type)
{
	DEB_MEMBER_FUNCT();
	TUCAMRET err;
	TUCAM_ELEMENT node; // Property node
	node.nVal = type;
	node.pName = "SensorCooling";
	err = TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to set SensorCooling to the camera ! Error: " << err << " ";
	}
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::getSensorCooling(unsigned& type)
{
	DEB_MEMBER_FUNCT();
	TUCAM_ELEMENT node; // Property node
	int err = TUCAM_GenICam_ElementAttr(m_opCam.hIdxTUCam, &node, "SensorCooling");
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Unable to Read SensorCooling from the camera ! Error: " << err << " ";
	}
	type = node.nVal;
}

//-----------------------------------------------------
//
//----------------------------------------------------- 
std::string Camera::getAllParameters()
{
	DEB_MEMBER_FUNCT();
	std::stringstream res;
	TUCAM_ELEMENT node; // Property node
	node.pName = "Root";
	int err = TUCAM_GenICam_ElementAttrNext(m_opCam.hIdxTUCam, &node, node.pName);
	
	while(TUCAMRET_SUCCESS == err)
	{
		if (NULL == node.pName)
		{
			continue;
		}
		switch (node.Type)
		{
		case TU_ElemCategory:
			break;
		case TU_ElemBoolean:
			res << node.pName << " = " << node.nVal << std::endl;
			break;
		case TU_ElemInteger:
			res << node.pName << " = " << node.nVal << std::endl;
			break;
		case TU_ElemFloat:
			res << node.pName << " = " << node.dbVal << std::endl;
			break;
		case TU_ElemString:
			res << node.pName << " = " << node.pTransfer << std::endl;
			break;
		case TU_ElemEnumeration:
			res << node.pName << " = " << node.nVal << std::endl;
			break;
		case TU_ElemCommand:
			res << node.pName << " = " << node.nVal << std::endl;
			break;
		case TU_ElemRegister:
			res << node.pName << " = " << node.nVal << std::endl;
			break;
		
		default:
			res << "N/A" << std::endl;
			break;
		}
		
		err = TUCAM_GenICam_ElementAttrNext(m_opCam.hIdxTUCam, &node, node.pName);
	}
	return res.str();
}

//-----------------------------------------------------
//
//----------------------------------------------------- 
std::string Camera::getParameter(std::string parameter)
{
	DEB_MEMBER_FUNCT();
	std::stringstream res;
	TUCAM_ELEMENT node; // Property node
	int err = TUCAM_GenICam_ElementAttr(m_opCam.hIdxTUCam, &node, const_cast<char*>(parameter.c_str()));
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "getParameter: Unable to get " << parameter << " from the camera ! Error: " << err << " ";
	}
	res << node.pName << " = ";
	switch (node.Type)
	{
	case TU_ElemCategory:
		/* code */
		break;
	case TU_ElemBoolean:
		res << node.nVal;
		break;
	case TU_ElemInteger:
		res << node.nVal;
		break;
	case TU_ElemFloat:
		res << node.dbVal;
		break;
	case TU_ElemString:
		res << node.pTransfer;
		break;
	case TU_ElemEnumeration:
		res << node.nVal;
		break;
	case TU_ElemCommand:
		res << node.nVal;
		break;
	case TU_ElemRegister:
		/* code */
		break;
	
	default:
		res << "N/A";
		break;
	}
	return res.str();
}

//-----------------------------------------------------
//
//----------------------------------------------------- 
void Camera::setParameter(std::string parameter, double value)
{
	DEB_MEMBER_FUNCT();
	TUCAMRET err;
	TUCAM_ELEMENT node; // Property node
	err = TUCAM_GenICam_ElementAttr(m_opCam.hIdxTUCam, &node, const_cast<char*>(parameter.c_str()));
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Camera::setParameter: Unable to get " << parameter << " from the camera ! Error: " << err;
	}
	if (node.Access != 2 && node.Access != 4)
	{
		THROW_HW_ERROR(Error) << "Camera::setParameter: not writabale ! access type: " << node.Access;
	}
	node.nVal = value;
	switch (node.Type)
	{
	case TU_ElemCategory:
		/* code */
		break;
	case TU_ElemBoolean:
		node.nVal = value;
		break;
	case TU_ElemInteger:
		node.nVal = value;
		break;
	case TU_ElemFloat:
		node.dbVal = value;
		break;
	case TU_ElemEnumeration:
		node.nVal = value;
		break;
	case TU_ElemCommand:
		node.nVal = value;
		break;
	case TU_ElemRegister:
		/* code */
		break;
	
	default:
		THROW_HW_ERROR(Error) << "Camera::setParameter: not writabale !";
		break;
	}
	err = TUCAM_GenICam_SetElementValue(m_opCam.hIdxTUCam, &node);
	if(TUCAMRET_SUCCESS != err)
	{
		THROW_HW_ERROR(Error) << "Camera::setParameter: Unable to set " << parameter << " to the camera ! Error: " << err;
	}

}