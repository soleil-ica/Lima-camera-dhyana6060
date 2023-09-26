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
// DhyanaBinCtrlObj.h
// Created on: August 09, 2023
// Author: Marc DESGRANGES

#ifndef DHYANA6060BINCTRLOBJ_H
#define DHYANA6060BINCTRLOBJ_H

#include "lima/Debug.h"
#include "Dhyana6060Compatibility.h"
#include "lima/HwInterface.h"
#include "Dhyana6060Camera.h"

namespace lima
{
namespace Dhyana6060
{
class Camera;

/*******************************************************************
 * \class BinCtrlObj
 * \brief Control object providing Basler Bin interface
 *******************************************************************/
class LIBDHYANA_API BinCtrlObj : public HwBinCtrlObj
{
    DEB_CLASS_NAMESPC(DebModCamera, "BinCtrlObj", "Dhyana");
public:
    BinCtrlObj(Camera& cam);
    virtual ~BinCtrlObj();

    virtual void setBin(const Bin& bin);
    virtual void getBin(Bin& bin);
    //allow all binning
    virtual void checkBin(Bin& bin);
private:
    Camera& m_cam;

} ;

} // namespace Dhyana6060
} // namespace lima

#endif // DHYANA6060BINCTRLOBJ_H
