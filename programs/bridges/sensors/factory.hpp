#ifndef BRIDGES_SENSORS_FACTORY_HPP
#define BRIDGES_SENSORS_FACTORY_HPP

#include <DUNE/DUNE.hpp>

// Local headers.
#include "base.hpp"
#include "dev-cmre.hpp"
#include "dev-contros.hpp"
#include "dev-fluidion.hpp"
#include "dev-noc.hpp"
#include "dev-octopus.hpp"
#include "dev-sbe37.hpp"
#include "dev-turner.hpp"
#include "dev-iclisten.hpp"
#include "../pmc/sibBus.hpp"

//! String identifier to produce sensors.
enum SensorFactoryString
{
#define SENSOR(a, b)                            \
  SFS_ ## a,
#include "factory.def"
  SFS_NONE
};

static const char* c_strings[] = {
#define SENSOR(a, b)                            \
  #a,
#include "factory.def"
  "NONE"
};

//! Sensor Factory class.
class SensorFactory
{
public:
  //! Produce sensor device according with type
  //! and link with sensor interface board bus.
  //! @param[in] type sensor factory identifier string.
  //! @param[in] bus sensor interface board bus.
  //! @return pointer to new sensor object.
  static Sensor*
  produce(SensorFactoryString type, unsigned lane, SibBus& bus)
  {
    Sensor* sensor = NULL;

    unsigned baud = getBaudRate(type);

    switch (type)
    {
      case (SFS_CMRE):
        sensor = new DevCMRE(lane, &bus, baud);
        break;
      case (SFS_CONTROS):
        sensor = new DevContros(lane, &bus, baud);
        break;
      case (SFS_FLUIDION):
        sensor = new DevFluidion(lane, &bus, baud);
        break;
      case (SFS_NOC_AMMONIA):
        sensor = new DevNOC(lane, &bus, NPT_AMMONIA, baud);
        break;
      case (SFS_NOC_NITRATE):
        sensor = new DevNOC(lane, &bus, NPT_NITRATE, baud);
        break;
      case (SFS_NOC_PHOSPHATE):
        sensor = new DevNOC(lane, &bus, NPT_PHOSPHATE, baud);
        break;
      case (SFS_NOC_SILICATE):
        sensor = new DevNOC(lane, &bus, NPT_SILICATE, baud);
        break;
      case (SFS_OCTOPUS):
        sensor = new DevOctopus(lane, &bus, OM_ADVANCED, baud);
        break;
      case (SFS_SEABIRD):
        sensor = new DevSBE37(lane, &bus, baud);
        break;
      case (SFS_TURNER_CHLOROPHYLL):
        sensor = new DevTurner(lane, &bus, TPT_CHLOROPHYLL, baud);
        break;
      case (SFS_TURNER_TURBIDITY):
        sensor = new DevTurner(lane, &bus, TPT_TURBIDITY, 38400);
        break;
      case (SFS_TURNER_CRUDE_OIL):
        sensor = new DevTurner(lane, &bus, TPT_CRUDE_OIL, baud);
        break;
      case (SFS_TURNER_REFINED_OIL):
        sensor = new DevTurner(lane, &bus, TPT_REFINED_OIL, baud);
        break;
      case (SFS_TURNER_CDOM):
        sensor = new DevTurner(lane, &bus, TPT_CDOM, baud);
        break;
      case (SFS_ICLISTEN):
        sensor = new DevIcListen(lane, &bus, baud);
        break;
    }

    return sensor;
  }

  //! Get baud rate of sensor device according with type
  //! @param[in] type sensor factory identifier string.
  //! @return baud rate.
  static unsigned
  getBaudRate(SensorFactoryString type)
  {
    switch (type)
    {
#define SENSOR(a, b)                            \
      case SFS_ ## a: return b;
#include "factory.def"
    }
  }

  //! Get sensor section string.
  //! @param[in] type sensor factory identifier string.
  //! @return header string.
  static std::string
  getString(SensorFactoryString type)
  {
    return c_strings[type];
  }

  static SensorFactoryString
  fromString(const std::string& str)
  {
    if (str == "NONE")
      return SFS_NONE;
#define SENSOR(a,b )                            \
    else if (str == #a)                         \
      return SFS_ ## a;
#include "factory.def"

    return SFS_NONE;
  }
};

#endif