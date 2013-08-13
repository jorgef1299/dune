//***************************************************************************
// Copyright 2007-2013 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Universidade do Porto. For licensing   *
// terms, conditions, and further information contact lsts@fe.up.pt.        *
//                                                                          *
// European Union Public Licence - EUPL v.1.1 Usage                         *
// Alternatively, this file may be used under the terms of the EUPL,        *
// Version 1.1 only (the "Licence"), appearing in the file LICENCE.md       *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://www.lsts.pt/dune/licence.                                        *
//***************************************************************************
// Author: Ricardo Martins                                                  *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Actuators
{
  //! Device driver for MCD4R.
  //!
  //! MCD4R is a device that controls one robotic arm (pulse and
  //! finger), one pan & tilt, and one camera.
  //!
  //! @author Ricardo Martins
  namespace MCD4R
  {
    using DUNE_NAMESPACES;

    //! Packet identifiers.
    enum PacketIds
    {
      //! State.
      PKT_ID_STATE    = 1,
      //! Actuation command.
      PKT_ID_ACTUATE  = 2
    };

    //! Actuation commands.
    enum ActuateCommands
    {
      //! Camera pan.
      ACT_CAM_PAN    = 0,
      //! Camera tilt.
      ACT_CAM_TILT   = 1,
      //! Camera zoom.
      ACT_CAM_ZOOM   = 2,
      //! Camera focus.
      ACT_CAM_FOCUS  = 3,
      //! Camera exposure.
      ACT_CAM_EXPO   = 4,
      //! Camera pulse.
      ACT_ARM_PULSE  = 5,
      //! Camera finger.
      ACT_ARM_FINGER = 6
    };

    //! Camera pan directions
    enum PanCommands
    {
      //! Pan reverse
      PAN_DIR_RV     = 1,
      //! Pan stop
      PAN_DIR_ST     = 0,
      //! Pan forward
      PAN_DIR_FW     = -1
    };

    //! Camera tilt directions
    enum TiltCommands
    {
      //! Tilt reverse
      TILT_DIR_RV    = 1,
      //! Tilt stop
      TILT_DIR_ST    = 0,
      //! Tilt forward
      TILT_DIR_FW    = -1
    };

    //! Camera zoom commands
    enum ZoomCommands
    {
      //! Zoom decrease
      ZOOM_DEC       = -1,
      //! Zoom stop
      ZOOM_STOP      = 0,
      //! Zoom increase
      ZOOM_INC       = 1
    };

    //! Camera exposure commands
    enum ExposureCommands
    {
      //! Exposure decrease
      EXPO_DEC       = -1,
      //! Exposure stop
      EXPO_STOP      = 0,
      //! Exposure increase
      EXPO_INC       = 1
    };

    //! Arm pulse commands
    enum PulseCommands
    {
      //! Pulse forward
      PULSE_FW       = -1,
      //! Pulse stop
      PULSE_ST       = 0,
      //! Pulse reverse
      PULSE_RV       = 1
    };

    //! Arm finger commands
    enum FingerCommands
    {
      //! Finger forward
      FINGER_FW       = -1,
      //! Finger stop
      FINGER_ST       = 0,
      //! Finger reverse
      FINGER_RV       = 1
    };

    //! Board state parameters
    struct BoardState
    {
      //! ADC line 24 volt
      uint16_t v24;
      //! ADC line 12 volt
      uint16_t v12;
      //! ADC line 3.3 volt
      uint16_t v3_3;
      //! ADC line tilt eol
      uint16_t tilt_eol;
      //! ADC line itilt
      uint16_t itilt;
      //! ADC line pan eol
      uint16_t pan_eol;
      //! ADC line ipan
      uint16_t ipan;
      //! ADC line ipulse
      uint16_t ipulse;
      //! ADC line ifinger
      uint16_t ifinger;
      //! ADC line isys
      uint16_t isys;
    } __attribute__((packed));

    enum StateVoltages
    {
      //! 24V
      SV_24,
      //! 12V
      SV_12,
      //! 3.3V
      SV_3V3,
      //! Tilt EOL
      SV_TILT_EOL,
      //! Pan EOL
      SV_PAN_EOL,
      //! Number of voltages
      SV_TOTAL
    };

    enum StateCurrents
    {
      //! Tilt current
      SC_ITILT,
      //! Pan current
      SC_IPAN,
      //! Pulse current
      SC_IPULSE,
      //! Finger current
      SC_IFINGER,
      //! System current
      SC_ISYS,
      //! Total number of currents
      SC_TOTAL
    };

    //! Task arguments.
    struct Arguments
    {
      //! Laser name.
      std::string laser_name;
      //! Serial port device.
      std::string uart_dev;
      //! Watchdog timeout.
      double wdog_tout;
    };

    //! Amount of seconds to wait before restarting task.
    static const unsigned c_restart_delay = 1;
    //! Names for states voltages
    const char* c_voltage_labels[] =
    {
      "24V", "12V", "3V3", "TILT_EOL", "PAN_EOL"
    };
    //! Labels for state currents
    const char* c_current_labels[] =
    {
      "ITILT", "IPAN", "IPULSE", "IFINGER", "ISYS"
    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Control interface.
      UCTK::InterfaceUART* m_uart;
      //! Current.
      IMC::Current m_current[SC_TOTAL];
      //! Voltage.
      IMC::Voltage m_voltage[SV_TOTAL];
      //! MCU voltage.
      IMC::Voltage m_voltage_mcu;
      //! Watchdog.
      Counter<double> m_wdog;
      //! Task arguments.
      Arguments m_args;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_uart(NULL)
      {
        // Define configuration parameters.
        param("Serial Port - Device", m_args.uart_dev)
        .defaultValue("")
        .description("Serial port device used to communicate with the sensor");

        param("Watchdog Timeout", m_args.wdog_tout)
        .units(Units::Second)
        .defaultValue("2.0")
        .description("Watchdog timeout");

        param("Laser - Name", m_args.laser_name)
        .defaultValue("Laser")
        .description("Name of the laser");

        bind<IMC::SetLedBrightness>(this);
        bind<IMC::QueryLedBrightness>(this);
      }

      ~Task(void)
      { }

      void
      onEntityReservation(void)
      {
        for (unsigned i = 0; i < SV_TOTAL; ++i)
        {
          std::string vlabel = String::str("%s - %s", getEntityLabel(),
                                           c_voltage_labels[i]);
          m_voltage[i].setSourceEntity(reserveEntity(vlabel));
        }

        for (unsigned i = 0; i < SC_TOTAL; ++i)
        {
          std::string clabel = String::str("%s - %s", getEntityLabel(),
                                           c_current_labels[i]);
          m_current[i].setSourceEntity(reserveEntity(clabel));
        }
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        m_wdog.setTop(m_args.wdog_tout);
      }

      //! Acquire resources.
      void
      onResourceAcquisition(void)
      {
        try
        {
          m_uart = new UCTK::InterfaceUART(m_args.uart_dev);
          m_uart->open();
          UCTK::FirmwareInfo info = m_uart->getFirmwareInfo();
          if (info.isDevelopment())
            war("device is using unstable firmware");
          else
            inf("firmware version %u.%u.%u", info.major,
                info.minor, info.patch);
        }
        catch (std::runtime_error& e)
        {
          throw RestartNeeded(e.what(), c_restart_delay);
        }
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
        m_wdog.reset();
        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
        Memory::clear(m_uart);
      }

      bool
      actCommand(ActuateCommands cmd, int8_t dir)
      {
        UCTK::Frame frame;
        frame.setId(PKT_ID_ACTUATE);
        frame.setPayloadSize(1);
        frame.set((uint8_t)cmd, 0);
        frame.set((int)dir, 1);

        if (!m_uart->sendFrame(frame))
          return false;

        return true;
      }

      //! Pan camera forward, reverse or stop
      //! @param[in] dir direction to which it should pan
      //! @return true if successful in sending command
      inline bool
      cameraPan(PanCommands dir)
      {
        return actCommand(ACT_CAM_PAN, dir);
      }

      //! Tilt camera forward reverse or stop
      //! @param[in] dir direction to which it should tilt
      //! @return true if successful in sending command
      inline bool
      cameraTilt(TiltCommands dir)
      {
        return actCommand(ACT_CAM_TILT, dir);
      }

      //! Zoom camera
      //! @param[in] dir zoom direction
      //! @return true if successful in sending command
      inline bool
      cameraZoom(ZoomCommands dir)
      {
        return actCommand(ACT_CAM_ZOOM, dir);
      }

      //! Command camera's exposure
      //! @param[in] cmd exposure command
      //! @return true if successful in sending command
      inline bool
      cameraExposure(ExposureCommands cmd)
      {
        return actCommand(ACT_CAM_EXPO, cmd);
      }

      //! Command arm's pulse
      //! @param[in] dir arm's pulse direction
      //! @return true if successful in sending command
      inline bool
      armPulse(PulseCommands dir)
      {
        return actCommand(ACT_ARM_PULSE, dir);
      }

      //! Command arm's finger
      //! @param[in] dir arm's finger direction
      //! @return true if successful in sending command
      inline bool
      armFinger(FingerCommands dir)
      {
        return actCommand(ACT_ARM_FINGER, dir);
      }

      //! Get raw board state
      bool
      dispatchState(void)
      {
        UCTK::Frame frame;
        frame.setId(PKT_ID_STATE);

        if (!m_uart->sendFrame(frame))
          return false;

        if (frame.getPayloadSize() != sizeof(struct BoardState))
          return false;

        struct BoardState* ptr = (struct BoardState*)frame.getPayload();

        m_voltage[SV_24].value = ptr->v24;
        m_voltage[SV_12].value = ptr->v12;
        m_voltage[SV_3V3].value = ptr->v3_3;
        m_voltage[SV_TILT_EOL].value = ptr->tilt_eol;
        m_voltage[SV_PAN_EOL].value = ptr->pan_eol;

        m_current[SC_ITILT].value = ptr->itilt;
        m_current[SC_IPAN].value = ptr->ipan;
        m_current[SC_IPULSE].value = ptr->ipulse;
        m_current[SC_IFINGER].value = ptr->ifinger;
        m_current[SC_ISYS].value = ptr->isys;

        for (unsigned i = 0; i < SV_TOTAL; i++)
          dispatch(m_voltage[i]);

        for (unsigned i = 0; i < SC_TOTAL; i++)
          dispatch(m_current[i]);

        return true;
      }

      void
      consume(const IMC::SetLedBrightness* msg)
      {
        if (msg->name != m_args.laser_name)
          return;
      }

      void
      consume(const IMC::QueryLedBrightness* msg)
      {
        if (msg->name != m_args.laser_name)
          return;
      }

      //! Main loop.
      void
      onMain(void)
      {
        while (!stopping())
        {
          waitForMessages(1.0);

          if (m_wdog.overflow())
          {
            setEntityState(IMC::EntityState::ESTA_ERROR, Status::CODE_COM_ERROR);
            throw RestartNeeded(Status::getString(Status::CODE_COM_ERROR), c_restart_delay);
          }
          else
          {
            setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
          }
        }
      }
    };
  }
}

DUNE_TASK
