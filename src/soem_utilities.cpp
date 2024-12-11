#include "synapticon_ros2_control/soem_utilities.hpp"

#include <stdio.h>

#define EC_TIMEOUTMON 500

namespace synapticon_ros2_control {
OSAL_THREAD_FUNC ecatcheck(void * /*ptr*/) {
  int slave;
  uint8 currentgroup = 0;

  while (1) {
    if (soem_globals::kInOp &&
        ((soem_globals::kWkc < soem_globals::kExpectedWkc) ||
         ec_group[currentgroup].docheckstate)) {
      if (soem_globals::kNeedlf) {
        soem_globals::kNeedlf = FALSE;
        printf("\n");
      }
      /* one ore more slaves are not responding */
      ec_group[currentgroup].docheckstate = FALSE;
      ec_readstate();
      for (slave = 1; slave <= ec_slavecount; slave++) {
        if ((ec_slave[slave].group == currentgroup) &&
            (ec_slave[slave].state != EC_STATE_OPERATIONAL)) {
          ec_group[currentgroup].docheckstate = TRUE;
          if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
            printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n",
                   slave);
            ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
            ec_writestate(slave);
          } else if (ec_slave[slave].state == EC_STATE_SAFE_OP) {
            printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n",
                   slave);
            ec_slave[slave].state = EC_STATE_OPERATIONAL;
            ec_writestate(slave);
          } else if (ec_slave[slave].state > EC_STATE_NONE) {
            if (ec_reconfig_slave(slave, EC_TIMEOUTMON)) {
              ec_slave[slave].islost = FALSE;
              printf("MESSAGE : slave %d reconfigured\n", slave);
            }
          } else if (!ec_slave[slave].islost) {
            /* re-check state */
            ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
            if (ec_slave[slave].state == EC_STATE_NONE) {
              ec_slave[slave].islost = TRUE;
              printf("ERROR : slave %d lost\n", slave);
            }
          }
        }
        if (ec_slave[slave].islost) {
          if (ec_slave[slave].state == EC_STATE_NONE) {
            if (ec_recover_slave(slave, EC_TIMEOUTMON)) {
              ec_slave[slave].islost = FALSE;
              printf("MESSAGE : slave %d recovered\n", slave);
            }
          } else {
            ec_slave[slave].islost = FALSE;
            printf("MESSAGE : slave %d found\n", slave);
          }
        }
      }
      if (!ec_group[currentgroup].docheckstate)
        printf("OK : all slaves resumed OPERATIONAL.\n");
    }
    osal_usleep(10000);
  }
}
} // namespace synapticon_ros2_control
