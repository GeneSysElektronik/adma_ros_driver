# This message represents the decoded bitfields of all warning and error bytes

# error hardware
bool error_gyro_hw                     # 2501
bool error_accel_hw                    # 2502
bool error_ext_speed_hw                # 2503
bool error_gnss_hw                     # 2504

# error misc1
bool error_data_bus_checksum           # 2506
bool error_eeprom                      # 2507
bool error_xmit                        # 2508
bool error_cmd                         # 2509

# error misc2
bool error_data_bus                    # 2511
bool error_can_bus                     # 2512
bool error_num                         # 2514

# error misc3
bool error_temp_warning                # 2516
bool error_reduced_accuracy            # 2517
bool error_range_max                   # 2518

# warning GNSS
bool warn_gnss_no_solution             # 2521
bool warn_gnss_vel_ignored             # 2522
bool warn_gnss_pos_ignored             # 2523
bool warn_gnss_unable_to_cfg           # 2524

# warning misc1
bool warn_speed_off                    # 2526
bool warn_gnss_dualant_ignored         # 2528

bool error_hw_sticky                   # 2529
