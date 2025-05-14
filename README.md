# ppmac_rt_trajectory

This application enables **real-time trajectory execution** on **Power PMAC**, utilizing user-shared memory for trajectory input and position-following control via **electronic gearing**.

## Integration Instructions

To integrate the trajectory functionality into your Power PMAC project, follow these steps:

1. **Add the Header File**
    Assuming your project is located at:
    ```C:/Users/fedID/Desktop/projectname```
    Place the header file `rt_trajectory.h` into the following directory:
    ```C:/Users/fedID/Desktop/projectname/projectname/C Language/Realtime Routines```

    - Note: Depending on your development environment, you may need to escape spaces in the path. Here are two common methods:
        - Using backslashes (e.g., in Makefiles or scripts):
        ```C:/Users/fedID/Desktop/projectname/projectname/C\ Language/Realtime\ Routines```
        - Using quotes (e.g., in shell commands or build configurations):
        ```"C:/Users/fedID/Desktop/projectname/projectname/C Language/Realtime Routines"```
2. **Include the Header in Your Code**
    In the source file `usrcode.c`, include the header as shown below. Make sure to place it **below** the `pp_proj.h` include directive:

    ``` c
    #include "../Include/pp_proj.h"
    #include "../Include/rt_trajectory.h"
    ```
