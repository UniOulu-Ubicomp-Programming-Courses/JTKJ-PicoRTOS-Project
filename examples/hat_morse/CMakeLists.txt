# Remember to uncomment in the root CMakeLists.txt the corresponding add_subdirectory if you want to include this application in your project


add_executable(hat_morse
  ${CMAKE_CURRENT_LIST_DIR}/src/main.c
)

target_link_libraries(hat_morse PRIVATE
  pico_stdlib
#Comment FreeRTOS ones if you are not using main_freertos.c
  FreeRTOS-Kernel
  FreeRTOS-Kernel-Heap4
  usb_serial_debug
  TKJHAT_SDK
)

pico_enable_stdio_usb(hat_morse 0)
pico_enable_stdio_uart(hat_morse 0)

pico_add_extra_outputs(hat_morse)
