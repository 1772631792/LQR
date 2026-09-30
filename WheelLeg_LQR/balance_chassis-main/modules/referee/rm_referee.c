/**
 * @file rm_referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */

 #include "rm_referee.h"
 #include "bsp_log.h"
 #include "bsp_usart.h"
 #include "cmsis_os.h"
 #include "crc_ref.h"
 #include "daemon.h"
 #include "string.h"
 #include "task.h"
 
 #define RE_RX_BUFFER_SIZE 255u // 裁判系统接收缓冲区大小
 
 static USARTInstance *referee_usart_instance; // 裁判系统串口实例
 static DaemonInstance *referee_daemon;        // 裁判系统守护进程
 static referee_info_t referee_info;           // 裁判系统数据
 static Sentry_SendData_t sentry_send_data;
 #define SentryRevive 0  // 写死，读条复活为1不复活为0
 uint32_t BulletBuy = 0; // 哨兵在补给区弹丸的兑换量，初始化值即为第一次买的数量
 uint8_t SentrySeq = 0;
 /**
  * @brief  读取裁判数据,中断中读取保证速度
  * @param  buff: 读取到的裁判系统原始数据
  * @retval 是否对正误判断做处理
  * @attention  在此判断帧头和CRC校验,无误再写入数据，不重复判断帧头
  */
 static void JudgeReadData(uint8_t *buff)
 {
	 uint16_t judge_length; // 统计一帧数据长度
	 if (buff == NULL)      // 空数据包，则不作任何处理
		 return;
 
	 // 写入帧头数据(5-byte),用于判断是否开始存储裁判数据
	 memcpy(&referee_info.FrameHeader, buff, LEN_HEADER);
 
	 // 判断帧头数据(0)是否为0xA5
	 if (buff[SOF] == REFEREE_SOF)
	 {
		 // 帧头CRC8校验
		 if (Verify_CRC8_Check_Sum(buff, LEN_HEADER) == TRUE)
		 {
			 // 统计一帧数据长度(byte),用于CR16校验
			 judge_length = buff[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
			 // 帧尾CRC16校验
			 if (Verify_CRC16_Check_Sum(buff, judge_length) == TRUE)
			 {
				 // 2个8位拼成16位int
				 referee_info.CmdID = (buff[6] << 8 | buff[5]);
				 // 解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
				 // 第8个字节开始才是数据 data=7
				 switch (referee_info.CmdID)
				 {
					 case ID_game_state: // 0x0001
						 memcpy(&referee_info.GameState, (buff + DATA_Offset), LEN_game_state);
						 break;
					 case ID_game_result: // 0x0002
						 memcpy(&referee_info.GameResult, (buff + DATA_Offset), LEN_game_result);
						 break;
					 case ID_game_robot_survivors: // 0x0003
						 memcpy(&referee_info.GameRobotHP, (buff + DATA_Offset), LEN_game_robot_HP);
						 break;
					 case ID_event_data: // 0x0101
						 memcpy(&referee_info.EventData, (buff + DATA_Offset), LEN_event_data);
						 break;
					 case ID_referee_warning: // 0x0104
						 memcpy(&referee_info.RefereeWarning, (buff + DATA_Offset), LEN_referee_warning);
						 break;
					 case ID_dart_info: // 0x0105
						 memcpy(&referee_info.DartInfo, (buff + DATA_Offset), LEN_dart_info);
						 break;
					 case ID_game_robot_state: // 0x0201
						 memcpy(&referee_info.GameRobotState, (buff + DATA_Offset), LEN_game_robot_state);
						 break;
					 case ID_power_heat_data: // 0x0202
						 memcpy(&referee_info.PowerHeatData, (buff + DATA_Offset), LEN_power_heat_data);
						 break;
					 case ID_game_robot_pos: // 0x0203
						 memcpy(&referee_info.GameRobotPos, (buff + DATA_Offset), LEN_game_robot_pos);
						 break;
					 case ID_buff_musk: // 0x0204
						 memcpy(&referee_info.BuffMusk, (buff + DATA_Offset), LEN_buff_musk);
						 break;
					 case ID_robot_hurt: // 0x0206
						 memcpy(&referee_info.RobotHurt, (buff + DATA_Offset), LEN_robot_hurt);
						 break;
                     case ID_shoot_data:  // 0x0207
                         referee_info.bullet_cnt++;
						 memcpy(&referee_info.ShootData, (buff + DATA_Offset), LEN_shoot_data);
						 break;
					 case ID_projectile_allowance: // 0x0208
						 memcpy(&referee_info.PojectileAllowance, (buff + DATA_Offset), LEN_projectile_allowance);
						 break;
					 case ID_rfid_status: // 0x0209
						 memcpy(&referee_info.RfidStatus, (buff + DATA_Offset), LEN_rfid_status);
						 break;
					 case ID_dart_client_cmd: // 0x020A
						 memcpy(&referee_info.DartClientCmd, (buff + DATA_Offset), LEN_dart_client_cmd);
						 break;
					 case ID_ground_robot_pos: // 0x020B
						 memcpy(&referee_info.ground_robot_p_t, (buff + DATA_Offset), LEN_ground_robot_pos);
						 break;
					 case ID_radar_mark_progress: // 0x020C
						 memcpy(&referee_info.RadarMarkData, (buff + DATA_Offset), LEN_radar_mark_progress);
						 break;
					 case ID_sentry_autonomous_decision_sync: // 0x020D
						 memcpy(&referee_info.Sentry_Auto_info, (buff + DATA_Offset), LEN_sentry_autonomous_decision_sync);
						 break;
					 case ID_radar_autonomous_decision_sync: // 0x020E
						 memcpy(&referee_info.RadarInfo, (buff + DATA_Offset), LEN_radar_autonomous_decision_sync);
						 break;
					 case ID_student_interactive: // 0x0301   syhtodo接收代码未测试
						 memcpy(&referee_info.ReceiveData, (buff + DATA_Offset), LEN_receive_data);
						 break;
					 case ID_custom_controller_interactive_data: // 0x0302
						 memcpy(&referee_info.ReceiveData, (buff + DATA_Offset), LEN_receive_data);
						 break;
					 case ID_map_command: // 0x0303
						 memcpy(&referee_info.MapCommand, (buff + DATA_Offset), LEN_map_command);
						 break;
					 case ID_keyboard_mouse_control_data: // 0x0304
						 memcpy(&referee_info.KeyboardRemoteControl, (buff + DATA_Offset), LEN_keyboard_remote_control);
						 break;
				 }
			 }
		 }
		 // 首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,从而判断一个数据包是否有多帧数据
		 if (*(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_info.FrameHeader.DataLength + LEN_TAIL) == 0xA5) // buff+5+2+1+2
		 {                                                                                                        // 如果一个数据包出现了多帧数据,则再次调用解析函数,直到所有数据包解析完毕
			 JudgeReadData(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_info.FrameHeader.DataLength + LEN_TAIL);
		 }
	 }
 }
 
 /*裁判系统串口接收回调函数,解析数据 */
 static void RefereeRxCallback()
 {
	 DaemonReload(referee_daemon);
	 JudgeReadData(referee_usart_instance->recv_buff);
 }
 // 裁判系统丢失回调函数,重新初始化裁判系统串口
 static void RefereeLostCallback(void *arg)
 {
	 USARTServiceInit(referee_usart_instance);
	 LOGWARNING("[rm_ref] lost referee data");
 }
 
 /* 裁判系统通信初始化 */
 referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle)
 {
	 USART_Init_Config_s conf;
	 conf.module_callback = RefereeRxCallback;
	 conf.usart_handle = referee_usart_handle;
	 conf.recv_buff_size = RE_RX_BUFFER_SIZE; // mx 255(u8)
	 referee_usart_instance = USARTRegister(&conf);
 
	 Daemon_Init_Config_s daemon_conf = {
		 .callback = RefereeLostCallback,
		 .owner_id = referee_usart_instance,
		 .reload_count = 30, // 0.3s没有收到数据,则认为丢失,重启串口接收
	 };
	 referee_daemon = DaemonRegister(&daemon_conf);
 
	 return &referee_info;
 }
 
 /**
  * @brief 裁判系统数据发送函数
  * @param
  */
 void RefereeSend(uint8_t *send, uint16_t tx_len)
 {
	 USARTSend(referee_usart_instance, send, tx_len, USART_TRANSFER_DMA);
	 osDelay(115);
 }
 
 // 补给区买弹数量写死了，注意：这个必须要先执行裁判系统初始化（虽然这个应该不出意外不会出问题，吧）
 void SentryCmdSend()
 {
	 // 这些信息可以一次初始化完的，但是需要多写并调用一个函数，没必要
	 sentry_send_data.FrameHeader.SOF = 0XA5;
	 sentry_send_data.FrameHeader.DataLength = 10;
	 sentry_send_data.FrameHeader.Seq = SentrySeq > 255 ? 0 : SentrySeq++;
	 sentry_send_data.FrameHeader.CRC8 = Get_CRC8_Check_Sum((uint8_t *)&sentry_send_data, LEN_CRC8, 0xFF);
	 // Append_CRC8_Check_Sum((uint8_t *)&(sentry_send_data.FrameHeader), 5);
	 sentry_send_data.CmdID = ID_student_interactive;
	 sentry_send_data.datahead.data_cmd_id = 0x0120;
	 sentry_send_data.datahead.sender_ID = referee_info.GameRobotState.robot_id;
	 sentry_send_data.datahead.receiver_ID = Referee_Server;
 
	 // sentry_send_data.data = BulletBuy << 2;
	 sentry_send_data.data.base_redeem_bullet = BulletBuy; // 哨兵在补给点想要兑换的发弹量，花金币兑换，发送的发单量只能递增
	 sentry_send_data.data.reserved = 0;                   // 保留
	 sentry_send_data.frametail = Get_CRC16_Check_Sum((uint8_t *)&sentry_send_data, 17, 0xFFFF);
	 // RefereeSend((uint8_t *)&sentry_send_data, 19); // 发送
	 RefereeSend((uint8_t *)&sentry_send_data, 19);
 }
 // 标点买弹的话会用到的函数，里面包含了买弹发信息，不需要重复调用
 void SentryBulletAdd(uint32_t add_num)
 {
	 BulletBuy += add_num;
	 SentryCmdSend();
 }
 // 读条复活
 void Sentryrevive(uint8_t revive)
 {
	 if (revive == 1)
	 {
		 sentry_send_data.data.is_free_revive = 1; // 是否读条复活，0确定不复活，1确定复活。感觉可以写死。
												   // sentry_send_data.data.is_redeem_revive = 1;//立即复活
	 }
	 else
	 {
		 sentry_send_data.data.is_free_revive = 0; // 是否读条复活，0确定不复活，1确定复活
	 }
	 SentryCmdSend();
 }
 
 // // 发送给裁判系统，然后发送给雷达
 // void SentryCmdSendToRadar(uint16_t id, float x, float y)
 // {
 //     Sentry_SendData_ToRadar_t send_to_radar;
 //     // 这些信息可以一次初始化完的，但是需要多写并调用一个函数，没必要
 //     // 帧头设置
 //     send_to_radar.FrameHeader.SOF = 0XA5;
 //     send_to_radar.FrameHeader.DataLength = 16;
 //     send_to_radar.FrameHeader.Seq = SentrySeq > 255 ? 0 : SentrySeq++;
 //     // CRC8校验
 //     send_to_radar.FrameHeader.CRC8 = Get_CRC8_Check_Sum((uint8_t *)&sentry_send_data, LEN_CRC8, 0xFF);
 //     send_to_radar.CmdID = ID_student_interactive;
 //     send_to_radar.datahead.data_cmd_id = 0x0200;
 //     send_to_radar.datahead.sender_ID = referee_info.GameRobotState.robot_id;
 //     send_to_radar.datahead.receiver_ID = referee_info.GameRobotState.robot_id + 2;
 //     // 虽然很抽象，但是很简洁很好用
 //     send_to_radar.Data.Robot_ID = id;
 //     send_to_radar.Data.x = x;
 //     send_to_radar.Data.y = y;
 //     send_to_radar.frametail = Get_CRC16_Check_Sum((uint8_t *)&sentry_send_data, 23, 0xFFFF);
 //     RefereeSend((uint8_t *)&sentry_send_data, 25); // 发送
 // }