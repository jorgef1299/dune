//***************************************************************************
// Copyright 2007-2019 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Faculdade de Engenharia da             *
// Universidade do Porto. For licensing terms, conditions, and further      *
// information contact lsts@fe.up.pt.                                       *
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Miguel Lançós                                                    *
//***************************************************************************

// Local headers
#include "DriverAD5272.hpp"

namespace Sensors
{
  namespace DMSv2
  {
    using DUNE_NAMESPACES;

    /**
     * @brief This function allows to reset the AD5272 RDAC register to the Memory set 
     * value and the CTRL register to zero.
     * 
     * @return True if the response and communication is as expected.
     */
    bool
    DriverAD5272::doReset(void)
    {
      m_task->trace("DriverAD5272::doReset executing.");

      uint8_t recv_data;
      bool frame_error;

      if(request(CMD_AD5272_RESET, NULL, 0, &recv_data, 1, frame_error))
      {
        if(frame_error)
        {
          m_task->spew("[DriverAD5272::doReset] Received an error frame type.");
          return false;
        }

        if(recv_data == 1)
          return true;
      }

      m_task->spew("[DriverAD5272::doReset] Communication error.");
      return false;
    }

    /**
     * @brief This function allows to enable or disable the shutdown mode of the AD5272.
     * 
     * @param enable True to enable the Shutdown mode.
     * @return True if communication and response is as expected.
     */
    bool
    DriverAD5272::setShutdownMode(bool enable)
    {
      m_task->trace("DriverAD5272::setShutdownMode executing.");

      uint8_t recv_data;
      bool frame_error;

      if(request(CMD_AD5272_SHUTDOWN, (uint8_t*)&enable, 1, &recv_data, 1, frame_error))
      {
        if(frame_error)
        {
          m_task->spew("[DriverAD5272::setShutdownMode] Received an error frame type.");
          return false;
        }

        if(recv_data == 1)
          return true;
      }

      m_task->spew("[DriverAD5272::setShutdownMode] Communication error.");
      return false;
    }

    /**
     * @brief This function allows to read the contents of the CTRL register of
     * the AD5272.
     * 
     * @param ctrl_register Contents of the CTRL register.
     * @return True if communication and response is as expected.
     */
    bool
    DriverAD5272::readCTRL(uint16_t& ctrl_register)
    {
      m_task->trace("DriverAD5272::readCTRL executing.");

      uint8_t recv_data[2];
      bool frame_error;

      if(request(CMD_AD5272_READCTRL, NULL, 0, recv_data, 2, frame_error))
      {
        if(frame_error)
        {
          m_task->spew("[DriverAD5272::readCTRL] Received an error frame type.");
          return false;
        }

        if(recv_data[1] > 0x0F)
        {
          m_task->spew("[DriverAD5272::readCTRL] Received data is not expected.");
          return false;
        }

        ctrl_register = (recv_data[0]<<8) | recv_data[1];
        return true;
      }

      m_task->spew("[DriverAD5272::readCTRL] Communication error.");
      return false;
    }

    /**
     * @brief This function allows to write the CTRL register with the
     * desired settings.
     * 
     * @param mem_write Activate the writing in the 50-TP memory.
     * @param rdac_write Activate the writing in the RDAC register.
     * @param rcal  Activate the Resistor Calibration.
     * @return True if communication and response is as expected.
     */
    bool
    DriverAD5272::writeCTRL(bool mem_write, bool rdac_write, bool rcal)
    {
      m_task->trace("DriverAD5272::writeCTRL executing.");

      uint8_t recv_data, send_data;
      bool frame_error;

      send_data = ((uint8_t)rcal << 2) | ((uint8_t)rdac_write << 1) | (uint8_t)mem_write;
      if(request(CMD_AD5272_WRITECTRL, &send_data, 1, &recv_data, 1, frame_error))
      {
        if(frame_error)
        {
          m_task->spew("[DriverAD5272::writeCTRL] Received an error frame type.");
          return false;
        }

        if(recv_data == 1)
          return true;
      }

      m_task->spew("[DriverAD5272::writeCTRL] Communication error.");
      return false; 
    }

    /**
     * @brief This function allows to change one or more CTRL settings without altering
     * the other settings. The selected settings (TRUE) will change to the value present
     * in the @enable.
     * 
     * @param mem_write Activate the writing in the 50-TP memory.
     * @param rdac_write Activate the writing in the RDAC register.
     * @param rcal  Activate the Resistor Calibration.
     * @param enable Value to set the selected settings.
     * @return True if communication and response is as expected.
     */
    bool
    DriverAD5272::setCTRLSettings(bool mem_write, bool rdac_write, bool rcal, bool enable)
    {
      m_task->trace("DriverAD5272::setCTRLSettings executing.");

      uint8_t recv_data, send_data;
      bool frame_error;

      send_data = ((uint8_t)rcal << 3) | ((uint8_t)rdac_write << 2) | ((uint8_t)mem_write << 1) | (uint8_t)enable;
      if(request(CMD_AD5272_CTRLSETTING, &send_data, 1, &recv_data, 1, frame_error))
      {
        if(frame_error)
        {
          m_task->spew("[DriverAD5272::setCTRLSettings] Received an error frame type.");
          return false;
        }

        if(recv_data == 1)
          return true;
      }

      m_task->spew("[DriverAD5272::setCTRLSettings] Communication error.");
      return false; 
    }

    /**
     * @brief This function allows to read the current resistance value of the
     * AD5272 by reading its RDAC register.
     * 
     * @param resistor Current value of the resistance.
     * @return True if communication and response is as expected.
     */
    bool
    DriverAD5272::readResistance(int& resistor)
    {
      m_task->trace("DriverAD5272::readResistance executing.");

      uint8_t recv_data[2];
      bool frame_error;

      if(request(CMD_AD5272_READRDAC, NULL, 0, recv_data, 2, frame_error))
      {
        if(frame_error)
        {
          m_task->spew("[DriverAD5272::readResistance] Received an error frame type.");
          return false;
        }

        resistor = (recv_data[0]<<8) | recv_data[1];
        resistor = (resistor * c_max_resistor) / (1 << c_rdac_bits);
        return true;
      }

      m_task->spew("[DriverAD5272::readResistance] Communication error.");
      return false;
    }

    /**
     * @brief This function allows to change the AD5272 resistance by writing in 
     * the RDAC register.
     * 
     * @param resistor Desired resistance of AD5272.
     * @return True if communication and response is as expected and writing to the 
     * RDAC register is enabled.
     */
    bool
    DriverAD5272::changeResistance(int resistor)
    {
      m_task->trace("DriverAD5272::changeResistance executing.");

      uint8_t recv_data, send_data[2];
      uint16_t aux;
      bool frame_error;

      if(readCTRL(aux))
      {
        if(!(aux & 0x02))
        {
          m_task->spew("[DriverAD5272::changeResistance] RDAC writing is not active.");
          return false;
        }
      }
      else
      {
        m_task->spew("[DriverAD5272::changeResistance] Error reading the CTRL register.");
        return false;
      }

      send_data[0] = (resistor >> 8);
      send_data[1] = resistor;
      if(request(CMD_AD5272_WRITERDAC, send_data, 2, &recv_data, 1, frame_error))
      {
        if(frame_error)
        {
          m_task->spew("[DriverAD5272::changeResistance] Received an error frame type.");
          return false;
        }

        if(recv_data == 1)
          return true;
      }

      m_task->spew("[DriverAD5272::changeResistance] Communication error.");
      return false; 
    }

    /**
     * @brief This function allows to read the next 50-TP memory address to be written
     * on the AD5272. There is only 50 addresses.
     * 
     * @param mem_address Memory address.
     * @return True if communication and response is as expected.
     */
    bool
    DriverAD5272::readMemoryAddress(int& mem_address)
    {
      m_task->trace("DriverAD5272::readMemoryAddress executing.");

      uint8_t recv_data[2];
      bool frame_error;

      if(request(CMD_AD5272_READMEMADDR, NULL, 0, recv_data, 2, frame_error))
      {
        if(frame_error)
        {
          m_task->spew("[DriverAD5272::readMemoryAddress] Received an error frame type.");
          return false;
        }

        if(recv_data[1] > 0x32)
        {
          m_task->spew("[DriverAD5272::readMemoryAddress] Received data is not expected.");
          return false;
        }

        mem_address = (recv_data[0]<<8) | recv_data[1];
        return true;
      }

      m_task->spew("[DriverAD5272::readMemoryAddress] Communication error.");
      return false;
    }
  }
}