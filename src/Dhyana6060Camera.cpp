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
//#include <chrono>
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
	//TODO
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::startAcq()
{
	DEB_MEMBER_FUNCT();
	//TODO
}

void Camera::_startAcq()
{
  DEB_MEMBER_FUNCT();
  //TODO
}
//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::stopAcq()
{
	DEB_MEMBER_FUNCT();
	//TODO
}

//-----------------------------------------------------
// @brief set the new camera status
//-----------------------------------------------------
void Camera::setStatus(Camera::Status status, bool force)
{
	DEB_MEMBER_FUNCT();
	//TODO
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getStatus(Camera::Status& status)
{
	DEB_MEMBER_FUNCT();
	//TODO
}

//-----------------------------------------------------
//
//-----------------------------------------------------
bool Camera::readFrame(void *bptr, int& frame_nb)
{
	DEB_MEMBER_FUNCT();
	//TODO
	return true;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::AcqThread::threadFunction()
{
	DEB_MEMBER_FUNCT();
	//TODO
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
	//TODO
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setImageType(ImageType type)
{
	DEB_MEMBER_FUNCT();
	//TODO
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getDetectorType(std::string& type)
{
	DEB_MEMBER_FUNCT();
	//TODO	

}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getDetectorModel(std::string& model)
{
	DEB_MEMBER_FUNCT();
	//TODO	
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getDetectorImageSize(Size& size)
{
	DEB_MEMBER_FUNCT();
	//TODO
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getPixelSize(double& sizex, double& sizey)
{
	DEB_MEMBER_FUNCT();
	//TODO
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
	//TODO
	return true;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setTrigMode(TrigMode mode)
{
	DEB_MEMBER_FUNCT();
	//TODO
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getTrigMode(TrigMode& mode)
{
	DEB_MEMBER_FUNCT();
	//TODO
}

void Camera::getTriggerMode(TucamTriggerMode &mode)
{
	DEB_MEMBER_FUNCT();
	//TODO
}

void Camera::setTriggerMode(TucamTriggerMode mode)
{
	DEB_MEMBER_FUNCT();
	//TODO
}

void Camera::getTriggerEdge(TucamTriggerEdge &edge)
{
	DEB_MEMBER_FUNCT();
	//TODO
}

void Camera::setTriggerEdge(TucamTriggerEdge edge)
{
	DEB_MEMBER_FUNCT();
	//TODO
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getExpTime(double& exp_time)
{
	DEB_MEMBER_FUNCT();
	//TODO
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setExpTime(double exp_time)
{
	DEB_MEMBER_FUNCT();
	//TODO
}


//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setLatTime(double lat_time)
{
	DEB_MEMBER_FUNCT();
	//TODO
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getLatTime(double& lat_time)
{
	DEB_MEMBER_FUNCT();
	//TODO
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getExposureTimeRange(double& min_expo, double& max_expo) const
{
	DEB_MEMBER_FUNCT();
	//TODO
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getLatTimeRange(double& min_lat, double& max_lat) const
{
	DEB_MEMBER_FUNCT();
	//TODO
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setNbFrames(int nb_frames)
{
	DEB_MEMBER_FUNCT();
	//TODO
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getNbFrames(int& nb_frames)
{
	DEB_MEMBER_FUNCT();
	//TODO
}

//-----------------------------------------------------
//
//-----------------------------------------------------
int Camera::getNbHwAcquiredFrames()
{
	DEB_MEMBER_FUNCT();
	//TODO
	return 0;
}

//-----------------------------------------------------
// @brief range the binning to the maximum allowed
//-----------------------------------------------------
void Camera::checkBin(Bin &hw_bin)
{
	DEB_MEMBER_FUNCT();
	//TODO
}
//-----------------------------------------------------
// @brief set the new binning mode
//-----------------------------------------------------
void Camera::setBin(const Bin &set_bin)
{
	DEB_MEMBER_FUNCT();
	//TODO
}

//-----------------------------------------------------
// @brief return the current binning mode
//-----------------------------------------------------
void Camera::getBin(Bin &hw_bin)
{
	DEB_MEMBER_FUNCT();
	//TODO
}

//-----------------------------------------------------
//! Camera::checkRoi()
//-----------------------------------------------------
void Camera::checkRoi(const Roi& set_roi, Roi& hw_roi)
{
	DEB_MEMBER_FUNCT();
	//TODO
}

//---------------------------------------------------------------------------------------
//! Camera::getRoi()
//---------------------------------------------------------------------------------------
void Camera::getRoi(Roi& hw_roi)
{
	DEB_MEMBER_FUNCT();
	//TODO
}

//---------------------------------------------------------------------------------------
//! Camera::setRoi()
//---------------------------------------------------------------------------------------
void Camera::setRoi(const Roi& set_roi)
{
	DEB_MEMBER_FUNCT();
	//TODO
}

//-----------------------------------------------------
//
//-----------------------------------------------------
bool Camera::isAcqRunning() const
{
	DEB_MEMBER_FUNCT();
	//TODO
	return true;
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
	node.nVal = temp;
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
	temp = m_temperature_target;
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
	/*err = TUCAM_GenICam_ElementAttrNext(m_opCam.hIdxTUCam, &node, node.pName);
	while(TUCAMRET_SUCCESS == err)
	{
		if (NULL == node.pName)
		{
			continue;
		}
		
		DEB_TRACE() << node.pName;
		err = TUCAM_GenICam_ElementAttrNext(m_opCam.hIdxTUCam, &node, node.pName);
	}*/
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
	type = node.dbVal;
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::setFanSpeed(unsigned speed)
{
	DEB_MEMBER_FUNCT();
	TUCAMRET err;
	TUCAM_ELEMENT node; // Property node
	node.nVal = speed;
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
	type = node.dbVal;
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
	gain = node.dbVal;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getTucamVersion(std::string& version)
{
	DEB_MEMBER_FUNCT();
	//TODO
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::getFirmwareVersion(std::string& version)
{
	DEB_MEMBER_FUNCT();
	//TODO
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
// Set trigger outpout on selected port
//-----------------------------------------------------  
void Camera::setOutputSignal(int port, TucamSignal signal, TucamSignalEdge edge, int delay, int width)
{
	DEB_MEMBER_FUNCT();
	//TODO
}

//-----------------------------------------------------
// Get trigger outpout on selected port
//----------------------------------------------------- 
void Camera::getOutputSignal(int port, TucamSignal& signal, TucamSignalEdge& edge, int& delay, int& width)
{
  DEB_MEMBER_FUNCT();
  //TODO
}
