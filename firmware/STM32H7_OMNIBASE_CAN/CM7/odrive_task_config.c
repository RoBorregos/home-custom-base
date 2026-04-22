/**
 * @file    odrive_task_config.c  (incorporate into / replace StartODriveTask in main.c)
 *
 * OVERVIEW
 * ─────────
 * The serial protocol now carries two message types, differentiated by the
 * first integer token on each line:
 *
 *   Type 1 – Control  (existing)
 *       "1 <vx> <vy> <wz>\r\n"
 *       → ODriveCmdMsg with type = ODRIVE_CMD_SET_VEL
 *
 *   Type 2 – Configuration  (new)
 *       "2 <sub_type> <mask_hex> [params...]\r\n"
 *       → ODriveCmdMsg with type = sub_type, target_mask = mask
 *
 * STATE MACHINE
 * ─────────────
 *
 *   ┌──────────────┐  power-on
 *   │  SM_BOOT     │─────────────────────────────────────────────────────────┐
 *   └──────────────┘                                                         │
 *           │ (timer expired or CFG_STARTUP received)                        │
 *           ▼                                                                │
 *   ┌──────────────┐  Clear errors → Set ctrl mode → Set CLOSED_LOOP        │
 *   │  SM_STARTUP  │ ◄──────────────────────────────── Type-2 CFG_STARTUP   │
 *   └──────────────┘                                                         │
 *           │ startup done                                                   │
 *           ▼                                                                │
 *   ┌──────────────┐                                                         │
 *   │  SM_RUNNING  │ ◄──────────────────────────────── Type-1 control cmds  │
 *   └──────────────┘                                                         │
 *         │ │ │                                                               │
 *         │ │ └── Type-2 CFG_CLEAR_ERRORS   → Clear errors, stay running     │
 *         │ └──── Type-2 CFG_SET_LIMITS /                                    │
 *         │       CFG_SET_POS_GAIN /                                         │
 *         │       CFG_SET_VEL_GAINS / …   → Tune, stay running              │
 *         │                                                                  │
 *         └────── Type-2 CFG_STOP / CFG_REBOOT → SM_IDLE / SM_BOOT ────────┘
 *
 *   ┌──────────────┐
 *   │  SM_IDLE     │ ◄── Type-2 CFG_STOP  (all axes → IDLE state)
 *   └──────────────┘
 *           │ CFG_STARTUP received
 *           └──► SM_STARTUP
 *
 * WHAT CHANGES IN main.c / ODriveTask
 * ─────────────────────────────────────
 * 1. Add ODriveSMState enum (below).
 * 2. Update UART_RX_Task to parse the leading '1' / '2' token (see parser
 *    snippet below).
 * 3. Replace the body of StartODriveTask with the version shown here, which
 *    wraps ODrive_ProcessCommand in a switch on sm_state.
 *
 * MISSING CAN FUNCTIONS  (add to ODrive.c / ODrive.h)
 * ──────────────────────────────────────────────────────
 * Based on the full ODrive CAN protocol, the following commands are present
 * in the ODrive spec but NOT yet implemented in your ODrive.c:
 *
 *   ● Get_Temperature        (cmd_id 0x15) – RTR frame, replies with
 *                                             [FET_temp f32][Motor_temp f32]
 *   ● Set_Traj_Vel_Limit     (cmd_id 0x11) – [traj_vel_limit f32]
 *   ● Set_Traj_Accel_Limits  (cmd_id 0x12) – [accel_limit f32][decel_limit f32]
 *   ● Set_Traj_Inertia       (cmd_id 0x13) – [inertia f32]
 *   ● Get_Torques            (cmd_id 0x1C) – RTR, replies [torque_set f32][torque_est f32]
 *   ● Get_Powers             (cmd_id 0x1D) – RTR, replies [elec_power f32][mech_power f32]
 *   ● Set_Absolute_Position  (cmd_id 0x19) – [position f32]
 *   ● Estop                  (cmd_id 0x02) – 0-byte DATA frame (emergency stop)
 *   ● Get_Error              (cmd_id 0x03) – RTR, replies [active_errors u32][disarm_reason u32]
 *   ● RxSdo / TxSdo         (cmd_ids 0x04/0x05) – arbitrary parameter R/W
 *                                             (needed for advanced tuning via CAN)
 *
 * Stub implementations for all of the above are provided at the bottom of
 * this file to make them copy-paste-ready into ODrive.c.
 */

/* ═══════════════════════════════════════════════════════════════════════════
 *  1.  NEW TYPES  –  add to ODrive.h (or a new header odrive_sm.h)
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Top-level ODrive task state machine states.
 */
typedef enum {
    SM_BOOT    = 0,   /**< Waiting for startup command or auto-start timeout */
    SM_STARTUP = 1,   /**< Executing clear/mode/state sequence */
    SM_RUNNING = 2,   /**< Normal closed-loop operation */
    SM_IDLE    = 3,   /**< All axes commanded to IDLE */
} ODriveSMState;

/* Sub-type IDs for Type-2 messages – must match Python CFG_* constants */
typedef enum {
    ODRIVE_CFG_CLEAR_ERRORS  = 20,
    ODRIVE_CFG_SET_STATE     = 21,
    ODRIVE_CFG_SET_CTRL_MODE = 22,
    ODRIVE_CFG_SET_LIMITS    = 23,
    ODRIVE_CFG_SET_POS_GAIN  = 24,
    ODRIVE_CFG_SET_VEL_GAINS = 25,
    ODRIVE_CFG_STARTUP       = 26,
    ODRIVE_CFG_REBOOT        = 27,
    ODRIVE_CFG_SET_TORQUE    = 28,
    ODRIVE_CFG_STOP          = 29,
    ODRIVE_CFG_SET_INPUT_POS = 30,
} ODriveCfgSubType;

/* Extend ODriveCmdMsg.type to include the new config sub-types.
 * Existing values (1–5 used by original ODRIVE_CMD_*) must be kept intact. */


/* ═══════════════════════════════════════════════════════════════════════════
 *  2.  UART_RX_TASK PARSER  –  replace the sscanf block in start_UART_RX_Task
 * ═══════════════════════════════════════════════════════════════════════════ */

/*
 * Replace the existing single sscanf call:
 *
 *   parsed = sscanf(line_buf, "%lf %lf %lf", ...);
 *
 * with the following block:
 */

/*  ──────────────────────── begin snippet ──────────────────────────────── */

void UART_RX_ParseLine(const char *line_buf, ODriveCmdMsg *odrive_cmd,
                       osMessageQueueId_t UART_QueueHandle,
                       osMessageQueueId_t URX_2_CAN_QueueHandle)
{
    int msg_type = 0;
    osStatus_t qst;

    if (sscanf(line_buf, "%d", &msg_type) != 1) {
        printf("Parse error (no type): \"%s\"\r\n", line_buf);
        return;
    }

    /* ── Type 1: control command ──────────────────────────────────────── */
    if (msg_type == 1) {
        double vx = 0.0, vy = 0.0, wz = 0.0;
        int parsed = sscanf(line_buf, "%*d %lf %lf %lf", &vx, &vy, &wz);

        if (parsed == 3) {
            odrive_cmd->type           = ODRIVE_CMD_SET_VEL;
            odrive_cmd->target_mask    = 0x0F;
            odrive_cmd->robot_twist[0] = vx;
            odrive_cmd->robot_twist[1] = vy;
            odrive_cmd->robot_twist[2] = wz;
            for (int i = 0; i < 4; i++) odrive_cmd->torque_ff[i] = 0.0f;

            qst = osMessageQueuePut(UART_QueueHandle, odrive_cmd, 0, 0);
            if (qst != osOK) printf("Failed to queue ctrl to UTX\r\n");

            qst = osMessageQueuePut(URX_2_CAN_QueueHandle, odrive_cmd, 0, 0);
            if (qst != osOK) printf("Failed to queue ctrl to CAN\r\n");

        } else {
            printf("Type-1 parse fail: \"%s\"\r\n", line_buf);
        }

    /* ── Type 2: configuration command ───────────────────────────────── */
    } else if (msg_type == 2) {
        int sub_type = 0;
        unsigned int mask_u = 0x0F;

        if (sscanf(line_buf, "%*d %d %x", &sub_type, &mask_u) < 2) {
            printf("Type-2 parse fail (sub/mask): \"%s\"\r\n", line_buf);
            return;
        }

        ODriveCmdMsg cfg_cmd = {0};
        cfg_cmd.type        = (uint8_t)sub_type;
        cfg_cmd.target_mask = (uint8_t)(mask_u & 0x0F);

        /* Parse optional per-sub-type parameters */
        switch ((ODriveCfgSubType)sub_type) {

            case ODRIVE_CFG_SET_STATE: {
                int state = 0;
                sscanf(line_buf, "%*d %*d %*x %d", &state);
                cfg_cmd.axis_state = (uint8_t)state;
                break;
            }
            case ODRIVE_CFG_SET_CTRL_MODE: {
                int cm = 0, im = 0;
                sscanf(line_buf, "%*d %*d %*x %d %d", &cm, &im);
                cfg_cmd.control_mode = (uint8_t)cm;
                cfg_cmd.input_mode   = (uint8_t)im;
                break;
            }
            case ODRIVE_CFG_SET_LIMITS: {
                float vl = 0.0f, cl = 0.0f;
                sscanf(line_buf, "%*d %*d %*x %f %f", &vl, &cl);
                cfg_cmd.vel_limit  = vl;
                cfg_cmd.curr_limit = cl;
                break;
            }
            case ODRIVE_CFG_SET_POS_GAIN: {
                float pg = 0.0f;
                sscanf(line_buf, "%*d %*d %*x %f", &pg);
                cfg_cmd.pos_gain = pg;
                break;
            }
            case ODRIVE_CFG_SET_VEL_GAINS: {
                float vg = 0.0f, vi = 0.0f;
                sscanf(line_buf, "%*d %*d %*x %f %f", &vg, &vi);
                cfg_cmd.vel_gain     = vg;
                cfg_cmd.vel_int_gain = vi;
                break;
            }
            case ODRIVE_CFG_STARTUP: {
                int cm = 2, im = 1, st = 8;
                sscanf(line_buf, "%*d %*d %*x %d %d %d", &cm, &im, &st);
                cfg_cmd.control_mode = (uint8_t)cm;
                cfg_cmd.input_mode   = (uint8_t)im;
                cfg_cmd.axis_state   = (uint8_t)st;
                break;
            }
            case ODRIVE_CFG_SET_TORQUE: {
                float tq = 0.0f;
                sscanf(line_buf, "%*d %*d %*x %f", &tq);
                cfg_cmd.torque_ff[0] = cfg_cmd.torque_ff[1] =
                cfg_cmd.torque_ff[2] = cfg_cmd.torque_ff[3] = tq;
                break;
            }
            case ODRIVE_CFG_SET_INPUT_POS: {
                float pos = 0.0f, vff = 0.0f, tff = 0.0f;
                sscanf(line_buf, "%*d %*d %*x %f %f %f", &pos, &vff, &tff);
                cfg_cmd.input_pos_target = pos;
                cfg_cmd.input_pos_vel_ff = vff;
                cfg_cmd.input_pos_trq_ff = tff;
                break;
            }
            /* CLEAR_ERRORS, REBOOT, STOP need no extra params */
            default:
                break;
        }

        qst = osMessageQueuePut(URX_2_CAN_QueueHandle, &cfg_cmd, 0, 0);
        if (qst != osOK) printf("Failed to queue cfg to CAN\r\n");

    } else {
        printf("Unknown msg_type %d: \"%s\"\r\n", msg_type, line_buf);
    }
}

/*  ──────────────────────── end snippet ────────────────────────────────── */


/* ═══════════════════════════════════════════════════════════════════════════
 *  3.  EXTENDED ODriveCmdMsg  –  add the new fields to ODrive.h
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * Add these fields to the ODriveCmdMsg struct (alongside the existing ones):
 *
 *   float    vel_limit;
 *   float    curr_limit;
 *   float    pos_gain;
 *   float    vel_gain;
 *   float    vel_int_gain;
 *   float    input_pos_target;
 *   float    input_pos_vel_ff;
 *   float    input_pos_trq_ff;
 *
 */


/* ═══════════════════════════════════════════════════════════════════════════
 *  4.  EXTENDED ODrive_ProcessCommand  –  add cases for the new sub-types
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * Add the following cases inside the existing switch(cmd->type) in
 * ODrive_ProcessCommand().  The existing cases are left unchanged.
 */

/*  ──────────────────────── begin snippet ──────────────────────────────── */

/* Insert after the last existing case, before default: */

        /* ── CFG: clear errors ────────────────────────────────────────── */
        case ODRIVE_CFG_CLEAR_ERRORS:
        {
            for (uint8_t i = 0; i < num_odrives; i++) {
                if (!(cmd->target_mask & (1 << i))) continue;
                while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) osDelay(1);
                st = Clear_Errors(&odrives[i], tx);
                if (st != HAL_OK) printf("CFG:Clear_Errors failed axis %u\r\n", i);
            }
            break;
        }

        /* ── CFG: set axis state ──────────────────────────────────────── */
        case ODRIVE_CFG_SET_STATE:
        {
            for (uint8_t i = 0; i < num_odrives; i++) {
                if (!(cmd->target_mask & (1 << i))) continue;
                while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) osDelay(1);
                st = Set_Axis_Requested_State(&odrives[i], tx, cmd->axis_state);
                if (st != HAL_OK) printf("CFG:SetState failed axis %u\r\n", i);
            }
            break;
        }

        /* ── CFG: set controller mode ─────────────────────────────────── */
        case ODRIVE_CFG_SET_CTRL_MODE:
        {
            for (uint8_t i = 0; i < num_odrives; i++) {
                if (!(cmd->target_mask & (1 << i))) continue;
                while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) osDelay(1);
                st = Set_Controller_Modes(&odrives[i], tx,
                         (Control_Mode)cmd->control_mode,
                         (Input_Mode)cmd->input_mode);
                if (st != HAL_OK) printf("CFG:SetCtrlMode failed axis %u\r\n", i);
            }
            *current_ctrl_mode  = (Control_Mode)cmd->control_mode;
            *current_input_mode = (Input_Mode)cmd->input_mode;
            break;
        }

        /* ── CFG: set vel / current limits ───────────────────────────── */
        case ODRIVE_CFG_SET_LIMITS:
        {
            for (uint8_t i = 0; i < num_odrives; i++) {
                if (!(cmd->target_mask & (1 << i))) continue;
                while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) osDelay(1);
                st = Set_Limits(&odrives[i], tx, cmd->vel_limit, cmd->curr_limit);
                if (st != HAL_OK) printf("CFG:SetLimits failed axis %u\r\n", i);
            }
            break;
        }

        /* ── CFG: set position gain ───────────────────────────────────── */
        case ODRIVE_CFG_SET_POS_GAIN:
        {
            for (uint8_t i = 0; i < num_odrives; i++) {
                if (!(cmd->target_mask & (1 << i))) continue;
                while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) osDelay(1);
                st = Set_Position_Gain(&odrives[i], tx, cmd->pos_gain);
                if (st != HAL_OK) printf("CFG:SetPosGain failed axis %u\r\n", i);
            }
            break;
        }

        /* ── CFG: set velocity gains ──────────────────────────────────── */
        case ODRIVE_CFG_SET_VEL_GAINS:
        {
            for (uint8_t i = 0; i < num_odrives; i++) {
                if (!(cmd->target_mask & (1 << i))) continue;
                while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) osDelay(1);
                st = Set_Vel_Gains(&odrives[i], tx, cmd->vel_gain, cmd->vel_int_gain);
                if (st != HAL_OK) printf("CFG:SetVelGains failed axis %u\r\n", i);
            }
            break;
        }

        /* ── CFG: full startup sequence ───────────────────────────────── */
        case ODRIVE_CFG_STARTUP:
        {
            HAL_StatusTypeDef startup_st = ODrive_Startup(
                odrives, num_odrives, tx,
                (Control_Mode)cmd->control_mode,
                (Input_Mode)cmd->input_mode,
                cmd->axis_state);
            if (startup_st != HAL_OK)
                printf("CFG:Startup failed\r\n");
            else {
                *current_ctrl_mode  = (Control_Mode)cmd->control_mode;
                *current_input_mode = (Input_Mode)cmd->input_mode;
            }
            break;
        }

        /* ── CFG: reboot ──────────────────────────────────────────────── */
        case ODRIVE_CFG_REBOOT:
        {
            for (uint8_t i = 0; i < num_odrives; i++) {
                if (!(cmd->target_mask & (1 << i))) continue;
                while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) osDelay(1);
                st = Reboot_ODrive(&odrives[i], tx);
                if (st != HAL_OK) printf("CFG:Reboot failed axis %u\r\n", i);
            }
            break;
        }

        /* ── CFG: set input torque ────────────────────────────────────── */
        case ODRIVE_CFG_SET_TORQUE:
        {
            for (uint8_t i = 0; i < num_odrives; i++) {
                if (!(cmd->target_mask & (1 << i))) continue;
                while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) osDelay(1);
                st = Set_Input_Torque(&odrives[i], tx, cmd->torque_ff[i]);
                if (st != HAL_OK) printf("CFG:SetTorque failed axis %u\r\n", i);
            }
            break;
        }

        /* ── CFG: stop (vel=0 then IDLE) ─────────────────────────────── */
        case ODRIVE_CFG_STOP:
        {
            for (uint8_t i = 0; i < num_odrives; i++) {
                if (!(cmd->target_mask & (1 << i))) continue;
                while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) osDelay(1);
                Set_Input_Vel(&odrives[i], tx, 0.0f, 0.0f);
            }
            osDelay(50);
            for (uint8_t i = 0; i < num_odrives; i++) {
                if (!(cmd->target_mask & (1 << i))) continue;
                while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) osDelay(1);
                Set_Axis_Requested_State(&odrives[i], tx, IDLE);
            }
            break;
        }

        /* ── CFG: set input position ──────────────────────────────────── */
        case ODRIVE_CFG_SET_INPUT_POS:
        {
            if (*current_ctrl_mode != POSITION_CONTROL) {
                printf("CFG:SetInputPos rejected: not in POSITION_CONTROL\r\n");
                break;
            }
            for (uint8_t i = 0; i < num_odrives; i++) {
                if (!(cmd->target_mask & (1 << i))) continue;
                while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) osDelay(1);
                st = Set_Input_Pos(&odrives[i], tx,
                         cmd->input_pos_target,
                         (int16_t)(cmd->input_pos_vel_ff * 1000.0f),
                         (int16_t)(cmd->input_pos_trq_ff * 1000.0f));
                if (st != HAL_OK) printf("CFG:SetInputPos failed axis %u\r\n", i);
            }
            break;
        }

/*  ──────────────────────── end snippet ────────────────────────────────── */


/* ═══════════════════════════════════════════════════════════════════════════
 *  5.  ODRIVETASK WITH STATE MACHINE  –  replace StartODriveTask body
 * ═══════════════════════════════════════════════════════════════════════════ */

void StartODriveTask_WithSM(void *argument)
{
    printf("\nODrive Task (State Machine)\r\n");

    /* ── Robot / axis parameters (unchanged from your original) ──────── */
    const uint8_t num_odrives  = 4;
    const double  x_offset     = 0.3;
    const double  y_offset     = 0.3;
    const double  radius       = 0.1;
    const double  wheel_sign[4] = { -1.0, 1.0, -1.0, 1.0 };

    odrives[0].NODE_ID = 36;
    odrives[1].NODE_ID = 34;
    odrives[2].NODE_ID = 33;
    odrives[3].NODE_ID = 40;

    /* ── State machine variables ──────────────────────────────────────── */
    ODriveSMState sm_state = SM_BOOT;
    Control_Mode  current_ctrl_mode  = VELOCITY_CONTROL;
    Input_Mode    current_input_mode = PASSTHROUGH;

    /* Auto-start after boot_delay_ms if no CFG_STARTUP is received */
    const uint32_t boot_delay_ms = 3000;
    uint32_t boot_tick = osKernelGetTickCount();

    /* ── Odometry / telemetry ─────────────────────────────────────────── */
    double x = 0.0, y = 0.0, theta = 0.0;
    double u[4]     = {0.0};
    double q_dot[3] = {0.0};
    ODriveCmdMsg  cmd         = {0};
    ODriveTelemetryMsg telemetryMsg = {0};
    OdomData *odrive_odom = &telemetryMsg.odom;
    FDCAN_TXmsg tx = {0};

    uint32_t now, last_telem_tick = osKernelGetTickCount();
    const uint32_t telemetry_period = 10;  /* ms  → 100 Hz */
    osStatus_t qst;

    /* IMU */
    uint8_t usingIMU = 0;
    bno055_vector_t euler = {0,0,0,0};
    if (usingIMU) {
        bno055_assignI2C(&hi2c1);
        bno055_setup();
        bno055_setOperationModeNDOF();
    }

    osDelay(1);

    for (;;)
    {
        now = osKernelGetTickCount();

        /* ── IMU read ─────────────────────────────────────────────────── */
        if (usingIMU) euler = bno055_getVectorEuler();
        telemetryMsg.imu.yaw   = euler.x;
        telemetryMsg.imu.roll  = euler.y;
        telemetryMsg.imu.pitch = euler.z;

        /* ── Receive command (non-blocking, 2 ms wait) ────────────────── */
        qst = osMessageQueueGet(URX_2_CAN_QueueHandle, &cmd, NULL, 2);

        /* ═══════════════════════════════════════════════════════════════
         *  STATE MACHINE
         * ═══════════════════════════════════════════════════════════════ */
        switch (sm_state)
        {
            /* ─── SM_BOOT ─────────────────────────────────────────────── */
            case SM_BOOT:
            {
                /* Transition 1: explicit startup command received */
                if (qst == osOK && cmd.type == ODRIVE_CFG_STARTUP) {
                    printf("SM: BOOT→STARTUP (cmd)\r\n");
                    sm_state = SM_STARTUP;
                    ODrive_ProcessCommand(&cmd, odrives, num_odrives, &tx,
                        odrive_odom, x_offset, y_offset, radius,
                        &current_ctrl_mode, &current_input_mode,
                        (double*)wheel_sign,
                        telemetryMsg.IK_computed_wheel_speeds);
                    sm_state = SM_RUNNING;
                }
                /* Transition 2: auto-start after timeout */
                else if ((now - boot_tick) >= boot_delay_ms) {
                    printf("SM: BOOT→STARTUP (auto)\r\n");
                    sm_state = SM_STARTUP;
                    HAL_StatusTypeDef st = ODrive_Startup(
                        odrives, num_odrives, &tx,
                        VELOCITY_CONTROL, PASSTHROUGH, CLOSED_LOOP_CONTROL);
                    if (st == HAL_OK) {
                        current_ctrl_mode  = VELOCITY_CONTROL;
                        current_input_mode = PASSTHROUGH;
                        sm_state = SM_RUNNING;
                        printf("SM: STARTUP→RUNNING\r\n");
                    } else {
                        printf("SM: Startup failed, retrying\r\n");
                        sm_state = SM_BOOT;
                        boot_tick = osKernelGetTickCount(); /* retry */
                    }
                }
                break;
            }

            /* ─── SM_STARTUP (transient) ──────────────────────────────── */
            case SM_STARTUP:
                /* This state is entered then immediately exited inside
                 * SM_BOOT above.  It only exists as a label for clarity. */
                sm_state = SM_RUNNING;
                break;

            /* ─── SM_RUNNING ──────────────────────────────────────────── */
            case SM_RUNNING:
            {
                if (qst == osOK) {
                    /* Config sub-commands that cause a state transition */
                    if (cmd.type == ODRIVE_CFG_STOP) {
                        printf("SM: RUNNING→IDLE (stop)\r\n");
                        ODrive_ProcessCommand(&cmd, odrives, num_odrives, &tx,
                            odrive_odom, x_offset, y_offset, radius,
                            &current_ctrl_mode, &current_input_mode,
                            (double*)wheel_sign,
                            telemetryMsg.IK_computed_wheel_speeds);
                        sm_state = SM_IDLE;
                        break;
                    }
                    if (cmd.type == ODRIVE_CFG_REBOOT) {
                        printf("SM: RUNNING→BOOT (reboot)\r\n");
                        ODrive_ProcessCommand(&cmd, odrives, num_odrives, &tx,
                            odrive_odom, x_offset, y_offset, radius,
                            &current_ctrl_mode, &current_input_mode,
                            (double*)wheel_sign,
                            telemetryMsg.IK_computed_wheel_speeds);
                        sm_state = SM_BOOT;
                        boot_tick = osKernelGetTickCount();
                        break;
                    }
                    /* All other commands (control + config tuning) */
                    ODrive_ProcessCommand(&cmd, odrives, num_odrives, &tx,
                        odrive_odom, x_offset, y_offset, radius,
                        &current_ctrl_mode, &current_input_mode,
                        (double*)wheel_sign,
                        telemetryMsg.IK_computed_wheel_speeds);
                }
                break;
            }

            /* ─── SM_IDLE ─────────────────────────────────────────────── */
            case SM_IDLE:
            {
                if (qst == osOK && cmd.type == ODRIVE_CFG_STARTUP) {
                    printf("SM: IDLE→STARTUP\r\n");
                    ODrive_ProcessCommand(&cmd, odrives, num_odrives, &tx,
                        odrive_odom, x_offset, y_offset, radius,
                        &current_ctrl_mode, &current_input_mode,
                        (double*)wheel_sign,
                        telemetryMsg.IK_computed_wheel_speeds);
                    sm_state = SM_RUNNING;
                }
                break;
            }
        } /* end switch(sm_state) */

        /* ── Telemetry update @ 100 Hz ────────────────────────────────── */
        uint32_t delta_t = now - last_telem_tick;
        if (delta_t >= telemetry_period) {
            ODrive_UpdateTelemetryAndOdometry(
                odrives, num_odrives, &telemetryMsg, odrive_odom,
                &x, &y, &theta,
                x_offset, y_offset, radius,
                u, q_dot, delta_t, (double*)wheel_sign);

            ODrive_PushLatestTelemetry(CAN_2_UTX_QueueHandle, &telemetryMsg);
            last_telem_tick = now;
        }

        osDelay(1);
    }
}


/* ═══════════════════════════════════════════════════════════════════════════
 *  6.  MISSING CAN FUNCTIONS  –  add these to ODrive.c and declare in ODrive.h
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Command IDs not yet in your ODrive.h – add them to the enum/defines: */
#define ESTOP                  0x002
#define GET_ERROR              0x003
#define RXSDO                  0x004
#define TXSDO                  0x005
#define GET_TEMPERATURE        0x015
#define SET_TRAJ_VEL_LIMIT     0x011
#define SET_TRAJ_ACCEL_LIMITS  0x012
#define SET_TRAJ_INERTIA       0x013
#define SET_ABSOLUTE_POSITION  0x019
#define GET_TORQUES            0x01C
#define GET_POWERS             0x01D


/** @brief Emergency stop – 0-byte data frame, immediately disarms the axis. */
HAL_StatusTypeDef Estop(const Axis *axis, FDCAN_TXmsg *msg) {
    Set_TX_Param(&msg->header, axis->NODE_ID, ESTOP,
                 FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_0);
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &msg->header, msg->data);
}

/** @brief Request active_errors + disarm_reason (RTR). */
HAL_StatusTypeDef Get_Error(const Axis *axis, FDCAN_TXmsg *msg) {
    Set_TX_Param(&msg->header, axis->NODE_ID, GET_ERROR,
                 FDCAN_STANDARD_ID, FDCAN_REMOTE_FRAME, FDCAN_DLC_BYTES_0);
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &msg->header, msg->data);
}

/** @brief Request FET + motor temperatures (RTR). */
HAL_StatusTypeDef Get_Temperature(const Axis *axis, FDCAN_TXmsg *msg) {
    Set_TX_Param(&msg->header, axis->NODE_ID, GET_TEMPERATURE,
                 FDCAN_STANDARD_ID, FDCAN_REMOTE_FRAME, FDCAN_DLC_BYTES_0);
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &msg->header, msg->data);
}

/** @brief Set trajectory velocity limit (turns/s). */
HAL_StatusTypeDef Set_Traj_Vel_Limit(const Axis *axis, FDCAN_TXmsg *msg,
                                      float traj_vel_limit) {
    Set_TX_Param(&msg->header, axis->NODE_ID, SET_TRAJ_VEL_LIMIT,
                 FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_4);
    pack_f32(&msg->data[0], traj_vel_limit);
    msg->data[4] = 0; msg->data[5] = 0; msg->data[6] = 0; msg->data[7] = 0;
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &msg->header, msg->data);
}

/** @brief Set trajectory acceleration / deceleration limits (turns/s²). */
HAL_StatusTypeDef Set_Traj_Accel_Limits(const Axis *axis, FDCAN_TXmsg *msg,
                                          float accel_limit, float decel_limit) {
    Set_TX_Param(&msg->header, axis->NODE_ID, SET_TRAJ_ACCEL_LIMITS,
                 FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_8);
    pack_f32(&msg->data[0], accel_limit);
    pack_f32(&msg->data[4], decel_limit);
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &msg->header, msg->data);
}

/** @brief Set trajectory inertia (Nm / (turn/s²)). */
HAL_StatusTypeDef Set_Traj_Inertia(const Axis *axis, FDCAN_TXmsg *msg,
                                    float inertia) {
    Set_TX_Param(&msg->header, axis->NODE_ID, SET_TRAJ_INERTIA,
                 FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_4);
    pack_f32(&msg->data[0], inertia);
    msg->data[4] = 0; msg->data[5] = 0; msg->data[6] = 0; msg->data[7] = 0;
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &msg->header, msg->data);
}

/** @brief Set absolute position reference (turns). */
HAL_StatusTypeDef Set_Absolute_Position(const Axis *axis, FDCAN_TXmsg *msg,
                                         float position) {
    Set_TX_Param(&msg->header, axis->NODE_ID, SET_ABSOLUTE_POSITION,
                 FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_4);
    pack_f32(&msg->data[0], position);
    msg->data[4] = 0; msg->data[5] = 0; msg->data[6] = 0; msg->data[7] = 0;
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &msg->header, msg->data);
}

/** @brief Request torque_setpoint + torque_estimate (RTR). */
HAL_StatusTypeDef Get_Torques(const Axis *axis, FDCAN_TXmsg *msg) {
    Set_TX_Param(&msg->header, axis->NODE_ID, GET_TORQUES,
                 FDCAN_STANDARD_ID, FDCAN_REMOTE_FRAME, FDCAN_DLC_BYTES_0);
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &msg->header, msg->data);
}

/** @brief Request electrical_power + mechanical_power (RTR). */
HAL_StatusTypeDef Get_Powers(const Axis *axis, FDCAN_TXmsg *msg) {
    Set_TX_Param(&msg->header, axis->NODE_ID, GET_POWERS,
                 FDCAN_STANDARD_ID, FDCAN_REMOTE_FRAME, FDCAN_DLC_BYTES_0);
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &msg->header, msg->data);
}

/*
 * RxSdo / TxSdo  – arbitrary parameter access.
 * These are more complex because they carry an endpoint_id and the value
 * in a protocol-buffer-style encoding.  Implement only if you need remote
 * parameter writing over CAN (e.g. motor.config.pole_pairs).
 * The full encoding is documented at:
 *   https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#rxsdo
 */


/* ═══════════════════════════════════════════════════════════════════════════
 *  7.  ODrive_RX_CallBack additions – handle new response types
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * Add these cases to the existing switch(cmd_id) in ODrive_RX_CallBack().
 * You also need to add the corresponding fields to the Axis struct:
 *
 *   float AXIS_FET_Temperature;
 *   float AXIS_Motor_Temperature;
 *   float AXIS_Torque_Setpoint;
 *   float AXIS_Torque_Estimate;
 *   float AXIS_Electrical_Power;
 *   float AXIS_Mechanical_Power;
 *   uint32_t AXIS_Disarm_Reason;
 */

        case GET_TEMPERATURE:
            axis->AXIS_FET_Temperature   = unpack_f32(&RX[0]);
            axis->AXIS_Motor_Temperature = unpack_f32(&RX[4]);
            axis->UPDATED = 1;
            break;

        case GET_TORQUES:
            axis->AXIS_Torque_Setpoint = unpack_f32(&RX[0]);
            axis->AXIS_Torque_Estimate = unpack_f32(&RX[4]);
            axis->UPDATED = 1;
            break;

        case GET_POWERS:
            axis->AXIS_Electrical_Power = unpack_f32(&RX[0]);
            axis->AXIS_Mechanical_Power = unpack_f32(&RX[4]);
            axis->UPDATED = 1;
            break;

        case GET_ERROR:
            axis->AXIS_Error        = unpack_u32(&RX[0]);
            axis->AXIS_Disarm_Reason = unpack_u32(&RX[4]);
            axis->UPDATED = 1;
            break;
