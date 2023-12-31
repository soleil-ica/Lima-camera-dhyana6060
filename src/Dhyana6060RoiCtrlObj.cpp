//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2018
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

#include "Dhyana6060Camera.h"
#include "Dhyana6060RoiCtrlObj.h"


using namespace lima;
using namespace lima::Dhyana6060;
/*******************************************************************
 * \brief RoiCtrlObj constructor
 *******************************************************************/

RoiCtrlObj::RoiCtrlObj(Camera& cam):m_cam(cam)
{
	DEB_CONSTRUCTOR();

}

//-----------------------------------------------------
//
//-----------------------------------------------------
RoiCtrlObj::~RoiCtrlObj()
{
	DEB_DESTRUCTOR();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void RoiCtrlObj::checkRoi(const Roi& set_roi, Roi& hw_roi)
{
	DEB_MEMBER_FUNCT();
	m_cam.checkRoi(set_roi, hw_roi);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void RoiCtrlObj::setRoi(const Roi& roi)
{
	DEB_MEMBER_FUNCT();
	Roi real_roi;
	checkRoi(roi, real_roi);
	m_cam.setRoi(real_roi);

}

//-----------------------------------------------------
//
//-----------------------------------------------------
void RoiCtrlObj::getRoi(Roi& roi)
{
	DEB_MEMBER_FUNCT();
	m_cam.getRoi(roi);
}
