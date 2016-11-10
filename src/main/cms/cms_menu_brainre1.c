/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

// Menu contents for PID, RATES, RC preview, misc
// Should be part of the relevant .c file.

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#ifdef CMS

#include "build/version.h"

#include "drivers/system.h"

//#include "common/typeconversion.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_imu.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/pid.h"

#include "config/config_profile.h"
#include "config/config_master.h"
#include "config/feature.h"

#include "target/BRAINRE1/video.h"
#include "target/BRAINRE1/osd_utils.h"
#include "target/BRAINRE1/ir_transponder.h"
#include "target/BRAINRE1/brainfpv_osd.h"


OSD_UINT8_t entryAhiSteps =  {&masterConfig.bfOsdConfig.ahi_steps, 0, 4, 1};
const char *FONT_NAMES[] = {"DEFAULT", "LARGE", "BOLD"};
OSD_TAB_t entryOSDFont = {&masterConfig.bfOsdConfig.font, 2, &FONT_NAMES[0]};
OSD_UINT8_t entryWhiteLevel =  {&masterConfig.bfOsdConfig.white_level, 100, 120, 1};
OSD_UINT8_t entryBlackLevel =  {&masterConfig.bfOsdConfig.black_level, 15, 40, 1};
OSD_UINT8_t entrySyncTh =  {&masterConfig.bfOsdConfig.sync_threshold, 110, 130, 1};
OSD_INT8_t entryXoff =  {&masterConfig.bfOsdConfig.x_offset, -8, 7, 1};
OSD_UINT8_t entryXScale =  {&masterConfig.bfOsdConfig.x_scale, 0, 15, 1};
OSD_UINT8_t entry3DShift =  {&masterConfig.bfOsdConfig.sbs_3d_right_eye_offset, 10, 40, 1};


OSD_Entry cmsx_menuBrainRE1OsdEntries[] =
{
    {"------- OSD --------", OME_Label, NULL, NULL},
    {"AHI STEPS", OME_UINT8, NULL, &entryAhiSteps},
    {"FONT", OME_TAB, NULL, &entryOSDFont},
    {"OSD WHITE", OME_UINT8, NULL, &entryWhiteLevel},
    {"OSD BLACK", OME_UINT8, NULL, &entryBlackLevel},
    {"OSD SYNC TH", OME_UINT8, NULL, &entrySyncTh},
    {"OSD X OFF", OME_INT8, NULL, &entryXoff},
    {"OSD X SC", OME_UINT8, NULL, &entryXScale},
    {"3D MODE", OME_Bool, NULL, &masterConfig.bfOsdConfig.sbs_3d_enabled},
    {"3D R SHIFT", OME_UINT8, NULL, &entry3DShift},

    {"BACK", OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL}
};

CMS_Menu cmsx_menuBrainRE1Osd = {
    .GUARD_text = "MENURE1OSD",
    .GUARD_type = OME_MENU,
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = cmsx_menuBrainRE1OsdEntries,
};


const char * IR_NAMES[] = {"OFF", "I-LAP", "TRACKMATE"};
OSD_TAB_t entryIRSys = {&masterConfig.bfOsdConfig.ir_system, 2, &IR_NAMES[0]};
OSD_UINT32_t entryIRIlap =  {&masterConfig.bfOsdConfig.ir_ilap_id, 0, 9999999, 1};
OSD_UINT16_t entryIRTrackmate =  {&masterConfig.bfOsdConfig.ir_trackmate_id, 0, 4095, 1};

OSD_Entry cmsx_menuBrainRE1IrEntries[] =
{
    {"-- IR TRANSPONDER --", OME_Label, NULL, NULL},
    {"IR SYS", OME_TAB, NULL, &entryIRSys},
    {"I LAP ID", OME_UINT32, NULL, &entryIRIlap},
    {"TRACKMATE ID", OME_UINT16, NULL, &entryIRTrackmate},

    {"BACK", OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL}
};

CMS_Menu cmsx_menuBrainRE1Ir = {
    .GUARD_text = "MENURE1IR",
    .GUARD_type = OME_MENU,
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = cmsx_menuBrainRE1IrEntries,
};

OSD_Entry cmsx_menuBrainRE1Entires[] =
{
    {"--- BRAINFPV RE1 ---", OME_Label, NULL, NULL},
    {"OSD", OME_Submenu, cmsMenuChange, &cmsx_menuBrainRE1Osd, 0},

    {"IR TRANSPONDER", OME_Submenu, cmsMenuChange, &cmsx_menuBrainRE1Ir, 0},
#if defined(USE_BRAINRE1_SPECTROGRAPH)
    {"SPECTROGRAPH", OME_Bool, NULL, &masterConfig.bfOsdConfig.spec_enabled},
#endif /* defined(USE_BRAINRE1_SPECTROGRAPH) */
    {"BACK", OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL}
};

CMS_Menu cmsx_menuBrainRE1 = {
    .GUARD_text = "MENURE1",
    .GUARD_type = OME_MENU,
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = cmsx_menuBrainRE1Entires,
};
#endif // CMS
