#include "robot.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(sl_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

using namespace Data_Functions;

namespace Lidar_Functions{
    void print_usage(int argc, const char * argv[])
    { 
        printf("Simple LIDAR data grabber for SLAMTEC LIDAR.\n"
            "Version: %s \n"
            "Usage:\n"
            " For serial channel %s --channel --serial <com port> [baudrate]\n"
            " \nJohann do this: ./johann_code --channel --serial /dev/ttyUSB0 115200\n\n"
            "The baudrate is 115200(for A2) or 256000(for A3).\n"
            " For udp channel %s --channel --udp <ipaddr> [port NO.]\n"
            "The LPX default ipaddr is 192.168.11.2,and the port NO.is 8089. Please refer to the datasheet for details.\n"
            , "SL_LIDAR_SDK_VERSION", argv[0], argv[0]);
    }

    bool checkSLAMTECLIDARHealth(ILidarDriver * drv)
    {
        sl_result     op_result;
        sl_lidar_response_device_health_t healthinfo;

        op_result = drv->getHealth(healthinfo);
        if (SL_IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
            printf("SLAMTEC Lidar health status : %d\n", healthinfo.status);
            if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
                fprintf(stderr, "Error, slamtec lidar internal error detected. Please reboot the device to retry.\n");
                // enable the following code if you want slamtec lidar to be reboot by software
                // drv->reset();
                return false;
            } else {
                return true;
            }

        } else {
            fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
            return false;
        }
    }

    bool ctrl_c_pressed;
    void ctrlc(int)
    {
        ctrl_c_pressed = true;
    }

    //main(int argc, const char * argv[]) 

    int runLidar(vector<PolPoint>& lidarDataPoints, bool& error){
        int argc = 5;
        const char * argv[] = {
            "./johann_code",
            "--channel",
            "--serial",
            "/dev/ttyUSB0",
            "115200"
        };
        bool newScan = true;
        // int NoPoints = 10000;
        // int NoPointsPerScan = 100000;
        int NoPoints = 8192;
        int NoPointsPerScan = 8192;
        sl_u16 stop = 0;
        sl_u32 timeout = 3000;//Default is 2000 (does not seem to help)
        sl_u32 timeStop = 2000;

        const char * opt_is_channel = NULL; 
        const char * opt_channel = NULL;
        const char * opt_channel_param_first = NULL;
        sl_u32         opt_channel_param_second = 0;
        sl_u32         baudrateArray[2] = {115200, 256000};
        sl_result     op_result;
        int          opt_channel_type = CHANNEL_TYPE_SERIALPORT;

        bool useArgcBaudrate = false;

        IChannel* _channel;

        // printf("Ultra simple LIDAR data grabber for SLAMTEC LIDAR.\n"
        //     "Version: %s\n", "SL_LIDAR_SDK_VERSION");

        
        if (argc>1)
        { 
            opt_is_channel = argv[1];
        }
        else
        {
            print_usage(argc, argv);
            return -1;
        }

        if(strcmp(opt_is_channel, "--channel")==0){
            opt_channel = argv[2];
            if(strcmp(opt_channel, "-s")==0||strcmp(opt_channel, "--serial")==0)
            {
                // read serial port from the command line...
                opt_channel_param_first = argv[3];// or set to a fixed value: e.g. "com3"
                // read baud rate from the command line if specified...
                if (argc>4) opt_channel_param_second = strtoul(argv[4], NULL, 10);	
                useArgcBaudrate = true;
            }
            else if(strcmp(opt_channel, "-u")==0||strcmp(opt_channel, "--udp")==0)
            {
                // read ip addr from the command line...
                opt_channel_param_first = argv[3];//or set to a fixed value: e.g. "192.168.11.2"
                if (argc>4) opt_channel_param_second = strtoul(argv[4], NULL, 10);//e.g. "8089"
                opt_channel_type = CHANNEL_TYPE_UDP;
            }
            else
            {
                print_usage(argc, argv);
                return -1;
            }
        }
        else
        {
            print_usage(argc, argv);
            return -1;
        }

        if(opt_channel_type == CHANNEL_TYPE_SERIALPORT)
        {
            if (!opt_channel_param_first) {
    #ifdef _WIN32
            // use default com port
            opt_channel_param_first = "\\\\.\\com3";
    #elif __APPLE__
            opt_channel_param_first = "/dev/tty.SLAB_USBtoUART";
    #else
            opt_channel_param_first = "/dev/ttyUSB0";
    #endif
            }
        }

        
        // create the driver instance
        ILidarDriver * drv = *createLidarDriver();



        if (!drv) {
            fprintf(stderr, "insufficent memory, exit\n");
            exit(-2);
        }

        sl_lidar_response_device_info_t devinfo;
        bool connectSuccess = false;

        if(opt_channel_type == CHANNEL_TYPE_SERIALPORT){
            if(useArgcBaudrate){
                _channel = (*createSerialPortChannel(opt_channel_param_first, opt_channel_param_second));
                if (SL_IS_OK((drv)->connect(_channel))) {
                    op_result = drv->getDeviceInfo(devinfo,timeout);

                    if (SL_IS_OK(op_result)) 
                    {
                        connectSuccess = true;
                    }
                    else{
                        cout<<"Get Device Info Failed_1"<<endl;
                        delete drv;
                        drv = NULL;
                    }
                }
            }
        }


        if (!connectSuccess) {
            (opt_channel_type == CHANNEL_TYPE_SERIALPORT)?
                (fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
                    , opt_channel_param_first)):(fprintf(stderr, "Error, cannot connect to the specified ip addr %s.\n"
                    , opt_channel_param_first));
            
            error = true;
            goto on_finished;
        }

        // print out the device serial number, firmware and hardware version number..
        printf("SLAMTEC LIDAR S/N: ");
        for (int pos = 0; pos < 16 ;++pos) {
            printf("%02X", devinfo.serialnum[pos]);
        }


        // check health...
        if (!checkSLAMTECLIDARHealth(drv)) {
            error = true;
            cout<<"Bad health"<<endl;
            goto on_finished;
        }

        signal(SIGINT, ctrlc);
        



        //Speed setup I think
        // if(opt_channel_type == CHANNEL_TYPE_SERIALPORT)
        //     cout<<"Serial Detected Set Speed"<<endl;
        //     drv->setMotorSpeed();
        drv->setMotorSpeed();
        // start scan...
        drv->startScan(0,1);



        //This is the while loop that we wanna be working in
        // fetech result and print it out...
        cout<<"Entering scan loop"<<endl;
        while (lidarDataPoints.size()<NoPoints) {
            sl_lidar_response_measurement_node_hq_t nodes[8192]; //number of points
            //sl_lidar_response_measurement_node_hq_t nodes[NoPointsPerScan]; //number of points
            size_t   count = _countof(nodes);

            op_result = drv->grabScanDataHq(nodes, count,timeout);

            
            if (SL_IS_OK(op_result)) {
                drv->ascendScanData(nodes, count);
                for (int pos = 0; pos < (int)count ; ++pos) {

                    if(nodes[pos].quality>40){
                        PolPoint newPoint;
                        newPoint.angle = (nodes[pos].angle_z_q14 * 90.f) / 16384.f;
                        newPoint.distance = nodes[pos].dist_mm_q2/4.0f;
                        lidarDataPoints.push_back(newPoint);
                    }else{
                        //cout<<"lowQuality"<<endl;
                    }
                    
                }
                

            }else{
                cout<<"SL is not ok"<<endl;
                error = true;
                goto on_finished;
            }

            if (ctrl_c_pressed){ 
                printf("I am in break statement Lidar_function");
                error = false;
                drv->setMotorSpeed(stop);
                break;
            }
        }

        printf("I have reached max NoPoints in Lidar_function");
        error = false;
        drv->setMotorSpeed(stop);
        drv->stop();
        delay(200);
        if(opt_channel_type == CHANNEL_TYPE_SERIALPORT)
            drv->setMotorSpeed(0);
        // done!
    on_finished:
        
        if(drv) {
            delete drv;
            drv = NULL;
        }
        return 0;
    }


    void fetchScan(ILidarDriver * drv, sl_result op_result, vector<PolPoint>& lidarDataPoints,
                  int NoPoints, bool& error,sl_u32 timeout){
        //This is the while loop that we wanna be working in
        // fetech result and print it out...
        cout<<"Entering scan loop"<<endl;
        while (lidarDataPoints.size()<NoPoints) {
            sl_lidar_response_measurement_node_hq_t nodes[8192]; //number of points
            //sl_lidar_response_measurement_node_hq_t nodes[NoPointsPerScan]; //number of points
            size_t   count = _countof(nodes);

            op_result = drv->grabScanDataHq(nodes, count,timeout);

            
            if (SL_IS_OK(op_result)) {
                drv->ascendScanData(nodes, count);
                for (int pos = 0; pos < (int)count ; ++pos) {

                    if(nodes[pos].quality>40){
                        PolPoint newPoint;
                        newPoint.angle = (nodes[pos].angle_z_q14 * 90.f) / 16384.f;
                        newPoint.distance = nodes[pos].dist_mm_q2/4.0f;
                        lidarDataPoints.push_back(newPoint);
                    }
                }
                

            }else{
                cout<<"SL is not ok"<<endl;
                error = true;
                return;
            }

            if (ctrl_c_pressed){ 
                printf("I am in break statement Lidar_function");
                error = true;
                return;
            }
        }

        error = false;
        return;

    }



}