# This message represents the decoded bitfields of all warning and error bytes

# error hardware
uint8 error_gyro_hw                     # 2501
uint8 error_accel_hw                    # 2502
uint8 error_ext_speed_hw                # 2503
uint8 error_gnss_hw                     # 2504

# error misc1
uint8 error_data_bus_checksum           # 2506
uint8 error_eeprom                      # 2507
uint8 error_xmit                        # 2508
uint8 error_cmd                         # 2509

# error misc2
uint8 error_data_bus                    # 2511
uint8 error_can_bus                     # 2512
uint8 error_num                         # 2514

# error misc3
uint8 error_temp_warning                # 2516
uint8 error_reduced_accuracy            # 2517
uint8 error_range_max                   # 2518

# warning GNSS
uint8 warn_gnss_no_solution             # 2521
uint8 warn_gnss_vel_ignored             # 2522
uint8 warn_gnss_pos_ignored             # 2523
uint8 warn_gnss_unable_to_cfg           # 2524

# warning misc1
uint8 warn_speed_off                    # 2526
uint8 warn_gnss_dualant_ignored         # 2528

uint8 error_hw_sticky                   # 2529
