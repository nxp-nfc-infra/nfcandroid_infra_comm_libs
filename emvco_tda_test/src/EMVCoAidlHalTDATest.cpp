/******************************************************************************
 *
 *  Copyright 2023 NXP
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of NXP nor the names of its contributors may be used
 * to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

#define LOG_TAG "emvco_tda_test"

/**
 * @brief  This class is responsible for getting user input
 *         from user and starts and stops the loopback with TDA
 *
 **/

#include "EMVCoTDA.h"
#include <cstring>

using aidl::vendor::nxp::emvco::NxpEmvcoState;
using aidl::vendor::nxp::emvco::NxpEmvcoTDAInfo;
using aidl::vendor::nxp::emvco::tda::test::EMVCoTDA;

const int NFC_A_PASSIVE_POLL_MODE = 0;
const int NFC_B_PASSIVE_POLL_MODE = 1;
const int NFC_F_PASSIVE_POLL_MODE = 2;
const int NFC_VAS_PASSIVE_POLL_MODE = 3;
const int NFC_AB_PASSIVE_POLL_MODE_SUPPORTED = 3;
const int NFC_F_PASSIVE_POLL_MODE_SUPPORTED = 4;
const int NFC_ABF_PASSIVE_POLL_MODE_SUPPORTED = 7;
const int NFC_ABVAS_PASSIVE_POLL_MODE_SUPPORTED = 11;
const int NFC_ABFVAS_PASSIVE_POLL_MODE_SUPPORTED = 15;

/**
 * @brief  Listens for user interrupt and exits the application
 *
 **/
void signal_callback_handler(int signum) {
  ALOGI("%s Self test App abort requested, signum:%d", __func__, signum);
  EMVCoTDA::getEMVCoTDAInstance().stopTDALoopback();
  exit(signum);
}

/**
 * @brief  set the RF poll technologies for EMVCo CL
 *
 **/
void setRFTechnologyMode(int modeType, bool isSet) {
  if (isSet) {
    EMVCoTDA::pollingConfiguration =
        1 << modeType | EMVCoTDA::pollingConfiguration;
  } else {
    EMVCoTDA::pollingConfiguration =
        ~(1 << modeType) & EMVCoTDA::pollingConfiguration;
  }
  ALOGI("%s EMVCoTDA::pollingConfiguration:%d\n", __func__,
        EMVCoTDA::pollingConfiguration);
}


/**
 * @brief  Entry point of application. Get's the user input from user
 *         and starts the loopback application on seperate thread
 *
 **/
int main(int argc, char **argv) {
  ABinderProcess_startThreadPool();

  try {
    ALOGI("%s Entered %d arguments", __func__, argc);
    for (int i = 0; i < argc; ++i) {
      ALOGI("%s argv:", argv[i]);
    }
    if (argc > 3) {
      if (strstr(argv[2], "A") != 0 || strstr(argv[2], "a") != 0) {
        setRFTechnologyMode(NFC_A_PASSIVE_POLL_MODE, true);
      }
      if (strstr(argv[2], "B") != 0 || strstr(argv[2], "b") != 0) {
        setRFTechnologyMode(NFC_B_PASSIVE_POLL_MODE, true);
      }
      if (strstr(argv[2], "F") != 0 || strstr(argv[2], "f") != 0) {
        setRFTechnologyMode(NFC_F_PASSIVE_POLL_MODE, true);
      }
      if (strstr(argv[2], "V") != 0 || strstr(argv[2], "v") != 0) {
        setRFTechnologyMode(NFC_VAS_PASSIVE_POLL_MODE, true);
      }

      if (argv[3] != NULL) {
        std::string tdaSlotStr = std::string(argv[3]);
        if (0 == strcasecmp(tdaSlotStr.c_str(),
                            EMVCoTDA::CT_CARD_TDA_SLOT_NAME.c_str())) {
          printf("\n CT Slot selected \n ");
          EMVCoTDA::getEMVCoTDAInstance().setTDASlot(CT_CARD_TDA_ID);
        } else if (0 == strcasecmp(tdaSlotStr.c_str(),
                                   EMVCoTDA::SAM1_CARD_TDA_SLOT_NAME.c_str())) {
          printf("\n SAM1 Slot selected \n ");
          EMVCoTDA::getEMVCoTDAInstance().setTDASlot(SAM1_CARD_TDA_ID);
        } else if (0 == strcasecmp(tdaSlotStr.c_str(),
                                   EMVCoTDA::SAM2_CARD_TDA_SLOT_NAME.c_str())) {
          printf("\n SAM2 Slot selected \n ");
          EMVCoTDA::getEMVCoTDAInstance().setTDASlot(SAM2_CARD_TDA_ID);
        } else {
          printf("\nSelect atleast one polling technolgy and 1 CT/SAM1/SAM2 "
                 "slot to enable EMVCo mode\n"
                 "Example#1: \"./EMVCoAidlHalTDATest Type AB CT\" "
                 "will enable Type AB poll and do interaction with CT \n"
                 "Example#2: \"./EMVCoAidlHalTDATest Type AB SAM1\" "
                 "will enable Type AB poll and do interaction with SAM1 \n"
                 "Example#3: \"./EMVCoAidlHalTDATest Type AB SAM2\" "
                 "will enable Type AB poll and do interaction with SAM2 \n");
          return 0;
        }
      } else {
        printf("\nSelect atleast one polling technolgy and one CT/SAM1/SAM2 "
               "slot to enable EMVCo mode\n"
               "Example#1: \"./EMVCoAidlHalTDATest Type AB CT\" "
               "will enable Type AB poll and do interaction with CT \n"
               "Example#2: \"./EMVCoAidlHalTDATest Type AB SAM1\" "
               "will enable Type AB poll and do interaction with SAM1 \n"
               "Example#3: \"./EMVCoAidlHalTDATest Type AB SAM2\" "
               "will enable Type AB poll and do interaction with SAM2 \n");
        return 0;
      }
    } else {
      printf("\nSelect atleast one polling technolgy and one CT/SAM1/SAM2 slot "
             "to enable EMVCo mode\n"
             "Example#1: \"./EMVCoAidlHalTDATest Type AB CT\" "
             "will enable Type AB poll and do interaction with CT \n"
             "Example#2: \"./EMVCoAidlHalTDATest Type AB SAM1\" "
             "will enable Type AB poll and do interaction with SAM1 \n"
             "Example#3: \"./EMVCoAidlHalTDATest Type AB SAM2\" "
             "will enable Type AB poll and do interaction with SAM2 \n");
      return 0;
    }

    if (EMVCoTDA::pollingConfiguration == NFC_AB_PASSIVE_POLL_MODE_SUPPORTED ||
        EMVCoTDA::pollingConfiguration == NFC_F_PASSIVE_POLL_MODE_SUPPORTED ||
        EMVCoTDA::pollingConfiguration == NFC_ABF_PASSIVE_POLL_MODE_SUPPORTED ||
        EMVCoTDA::pollingConfiguration ==
            NFC_ABVAS_PASSIVE_POLL_MODE_SUPPORTED ||
        EMVCoTDA::pollingConfiguration ==
            NFC_ABFVAS_PASSIVE_POLL_MODE_SUPPORTED) {
      printf("\n Valid Technology selected for polling\n ");
    } else {
      printf("\n Select atleast one polling technolgy and one CT/SAM1/SAM2 "
             "slot to enable EMVCo mode\n"
             "Example#1: \"./EMVCoAidlHalTDATest Type AB CT\" "
             "will enable Type AB poll and do interaction with CT \n"
             "Example#2: \"./EMVCoAidlHalTDATest Type AB SAM1\" "
             "will enable Type AB poll and do interaction with SAM1 \n"
             "Example#3: \"./EMVCoAidlHalTDATest Type AB SAM2\" "
             "will enable Type AB poll and do interaction with SAM2 \n");
      return 0;
    }

    signal(SIGINT, signal_callback_handler);
    EMVCoTDA::getEMVCoTDAInstance().startTDALoopback();
  } catch (const std::length_error &e) {
    ALOGE("%s std::length_error", e.what());
  }
  ALOGI("TEST APP EXITED");
}