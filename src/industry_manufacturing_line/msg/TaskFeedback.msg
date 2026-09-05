# 任务状态
uint8 TASK_STATUS_IDLE=0             # 闲置
uint8 TASK_STATUS_NAVIGATING=1       # 导航
uint8 TASK_STATUS_GRASPING=2         # 抓取
uint8 TASK_STATUS_TRANSPORTING=3     # 运输
uint8 TASK_STATUS_PLACING=4          # 放置
uint8 TASK_STATUS_SUCCESS=5         # 任务完成
uint8 TASK_STATUS_ERROR=6           # 任务异常


uint32 task_id                       # 任务编号
uint8 status                         # 任务状态值
float32 progress                     # 完成进度0-100
string error_code                    # 错误编码，无错误为空
string error_message                 # 错误详情
builtin_interfaces/Time stamp        # 消息时间戳
