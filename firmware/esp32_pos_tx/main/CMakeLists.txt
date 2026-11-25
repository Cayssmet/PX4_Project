# main 组件的 CMakeLists.txt
# 告诉构建系统：这个组件包含 main.c，头文件在当前目录
idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_wifi nvs_flash lwip)
