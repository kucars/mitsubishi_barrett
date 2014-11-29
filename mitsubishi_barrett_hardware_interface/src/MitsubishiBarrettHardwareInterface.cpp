#include <mitsubishi_barrett_hardware_interface/MitsubishiBarrettHardwareInterface.h>
MitsubishiBarrettHardwareInterface::MitsubishiBarrettHardwareInterface(const int& canbus_number_, bool& forcetorque_, bool& tactile_,  std::string& calibration_filename_, double& ft_pub_freq, double& tactile_pub_freq) : n("~"),
   canbus_number(canbus_number_),
   forcetorque(forcetorque_),
   tactile(tactile_),
   calibration_filename(calibration_filename_),
   ft_pub_freq_(ft_pub_freq),
   tactile_pub_freq_(tactile_pub_freq)
{
   pos.resize(joint_number);
   vel.resize(joint_number);
   eff.resize(joint_number);
   pos_cmd.resize(joint_number);
   vel_cmd.resize(joint_number);
   eff_cmd.resize(joint_number);
   pos_cmd_previous.resize(joint_number);
   vel_cmd_previous.resize(joint_number);
   eff_cmd_previous.resize(joint_number);

   // connect and register the joint state interface

   // MITSUBISHI ARM JOINTS
   hardware_interface::JointStateHandle mitsubishi_state_handle_j1("j1", &pos[0], &vel[0], &eff[0]);
   jnt_state_interface.registerHandle(mitsubishi_state_handle_j1);

   hardware_interface::JointStateHandle mitsubishi_state_handle_j2("j2", &pos[1], &vel[1], &eff[1]);
   jnt_state_interface.registerHandle(mitsubishi_state_handle_j2);

   hardware_interface::JointStateHandle mitsubishi_state_handle_j3("j3", &pos[2], &vel[2], &eff[2]);
   jnt_state_interface.registerHandle(mitsubishi_state_handle_j3);

   hardware_interface::JointStateHandle mitsubishi_state_handle_j4("j4", &pos[3], &vel[3], &eff[3]);
   jnt_state_interface.registerHandle(mitsubishi_state_handle_j4);

   hardware_interface::JointStateHandle mitsubishi_state_handle_j5("j5", &pos[4], &vel[4], &eff[4]);
   jnt_state_interface.registerHandle(mitsubishi_state_handle_j5);

   hardware_interface::JointStateHandle mitsubishi_state_handle_j6("j6", &pos[5], &vel[5], &eff[5]);
   jnt_state_interface.registerHandle(mitsubishi_state_handle_j6);

   // BARRETT HAND JOINTS
   hardware_interface::JointStateHandle barrett_state_handle_j1("finger_1_med_joint", &pos[0+mitsubishi_joint_number], &vel[0+mitsubishi_joint_number], &eff[0+mitsubishi_joint_number]);
   jnt_state_interface.registerHandle(barrett_state_handle_j1);

   hardware_interface::JointStateHandle barrett_state_handle_j2("finger_2_med_joint", &pos[1+mitsubishi_joint_number], &vel[1+mitsubishi_joint_number], &eff[1+mitsubishi_joint_number]);
   jnt_state_interface.registerHandle(barrett_state_handle_j2);

   hardware_interface::JointStateHandle barrett_state_handle_j3("finger_3_med_joint", &pos[2+mitsubishi_joint_number], &vel[2+mitsubishi_joint_number], &eff[2+mitsubishi_joint_number]);
   jnt_state_interface.registerHandle(barrett_state_handle_j3);

   hardware_interface::JointStateHandle barrett_state_handle_j4("finger_1_prox_joint", &pos[3+mitsubishi_joint_number], &vel[3+mitsubishi_joint_number], &eff[3+mitsubishi_joint_number]);
   jnt_state_interface.registerHandle(barrett_state_handle_j4);

   hardware_interface::JointStateHandle barrett_state_handle_j5("finger_2_prox_joint", &pos[4+mitsubishi_joint_number], &vel[4+mitsubishi_joint_number], &eff[4+mitsubishi_joint_number]);
   jnt_state_interface.registerHandle(barrett_state_handle_j5);

   hardware_interface::JointStateHandle barrett_state_handle_j6("finger_1_dist_joint", &pos[5+mitsubishi_joint_number], &vel[5+mitsubishi_joint_number], &eff[5+mitsubishi_joint_number]);
   jnt_state_interface.registerHandle(barrett_state_handle_j6);

   hardware_interface::JointStateHandle barrett_state_handle_j7("finger_2_dist_joint", &pos[6+mitsubishi_joint_number], &vel[6+mitsubishi_joint_number], &eff[6+mitsubishi_joint_number]);
   jnt_state_interface.registerHandle(barrett_state_handle_j7);

   hardware_interface::JointStateHandle barrett_state_handle_j8("finger_3_dist_joint", &pos[7+mitsubishi_joint_number], &vel[7+mitsubishi_joint_number], &eff[7+mitsubishi_joint_number]);
   jnt_state_interface.registerHandle(barrett_state_handle_j8);


   registerInterface(&jnt_state_interface);

   // connect and register the joint position interface
   // MITSUBISHI ARM
   hardware_interface::JointHandle mitsubishi_pos_handle_j1(jnt_state_interface.getHandle("j1"), &pos_cmd[0]);
   jnt_pos_interface.registerHandle(mitsubishi_pos_handle_j1);

   hardware_interface::JointHandle mitsubishi_pos_handle_j2(jnt_state_interface.getHandle("j2"), &pos_cmd[1]);
   jnt_pos_interface.registerHandle(mitsubishi_pos_handle_j2);

   hardware_interface::JointHandle mitsubishi_pos_handle_j3(jnt_state_interface.getHandle("j3"), &pos_cmd[2]);
   jnt_pos_interface.registerHandle(mitsubishi_pos_handle_j3);

   hardware_interface::JointHandle mitsubishi_pos_handle_j4(jnt_state_interface.getHandle("j4"), &pos_cmd[3]);
   jnt_pos_interface.registerHandle(mitsubishi_pos_handle_j4);

   hardware_interface::JointHandle mitsubishi_pos_handle_j5(jnt_state_interface.getHandle("j5"), &pos_cmd[4]);
   jnt_pos_interface.registerHandle(mitsubishi_pos_handle_j5);

   hardware_interface::JointHandle mitsubishi_pos_handle_j6(jnt_state_interface.getHandle("j6"), &pos_cmd[5]);
   jnt_pos_interface.registerHandle(mitsubishi_pos_handle_j6);

   // BARRETT HAND
   hardware_interface::JointHandle barrett_pos_handle_j1(jnt_state_interface.getHandle("finger_1_med_joint"), &pos_cmd[0+mitsubishi_joint_number]);
   jnt_pos_interface.registerHandle(barrett_pos_handle_j1);

   hardware_interface::JointHandle barrett_pos_handle_j2(jnt_state_interface.getHandle("finger_2_med_joint"), &pos_cmd[1+mitsubishi_joint_number]);
   jnt_pos_interface.registerHandle(barrett_pos_handle_j2);

   hardware_interface::JointHandle barrett_pos_handle_j3(jnt_state_interface.getHandle("finger_3_med_joint"), &pos_cmd[2+mitsubishi_joint_number]);
   jnt_pos_interface.registerHandle(barrett_pos_handle_j3);

   hardware_interface::JointHandle barrett_pos_handle_j4(jnt_state_interface.getHandle("finger_1_prox_joint"), &pos_cmd[3+mitsubishi_joint_number]);
   jnt_pos_interface.registerHandle(barrett_pos_handle_j4);

   hardware_interface::JointHandle barrett_pos_handle_j5(jnt_state_interface.getHandle("finger_2_prox_joint"), &pos_cmd[4+mitsubishi_joint_number]);
   jnt_pos_interface.registerHandle(barrett_pos_handle_j5);

   hardware_interface::JointHandle barrett_pos_handle_j6(jnt_state_interface.getHandle("finger_1_dist_joint"), &pos_cmd[5+mitsubishi_joint_number]);
   jnt_pos_interface.registerHandle(barrett_pos_handle_j6);

   hardware_interface::JointHandle barrett_pos_handle_j7(jnt_state_interface.getHandle("finger_2_dist_joint"), &pos_cmd[6+mitsubishi_joint_number]);
   jnt_pos_interface.registerHandle(barrett_pos_handle_j7);

   hardware_interface::JointHandle barrett_pos_handle_j8(jnt_state_interface.getHandle("finger_3_dist_joint"), &pos_cmd[7+mitsubishi_joint_number]);
   jnt_pos_interface.registerHandle(barrett_pos_handle_j8);


   registerInterface(&jnt_pos_interface);

   // connect and register the velocity interface
   hardware_interface::JointHandle barrett_vel_handle_j1(jnt_state_interface.getHandle("finger_1_med_joint"), &vel_cmd[0+mitsubishi_joint_number]);
   jnt_vel_interface.registerHandle(barrett_vel_handle_j1);

   hardware_interface::JointHandle barrett_vel_handle_j2(jnt_state_interface.getHandle("finger_2_med_joint"), &vel_cmd[1+mitsubishi_joint_number]);
   jnt_vel_interface.registerHandle(barrett_vel_handle_j2);

   hardware_interface::JointHandle barrett_vel_handle_j3(jnt_state_interface.getHandle("finger_3_med_joint"), &vel_cmd[2+mitsubishi_joint_number]);
   jnt_vel_interface.registerHandle(barrett_vel_handle_j3);

   hardware_interface::JointHandle barrett_vel_handle_j4(jnt_state_interface.getHandle("finger_1_prox_joint"), &vel_cmd[3+mitsubishi_joint_number]);
   jnt_vel_interface.registerHandle(barrett_vel_handle_j4);

   hardware_interface::JointHandle barrett_vel_handle_j5(jnt_state_interface.getHandle("finger_2_prox_joint"), &vel_cmd[4+mitsubishi_joint_number]);
   jnt_vel_interface.registerHandle(barrett_vel_handle_j5);

   hardware_interface::JointHandle barrett_vel_handle_j6(jnt_state_interface.getHandle("finger_1_dist_joint"), &vel_cmd[5+mitsubishi_joint_number]);
   jnt_vel_interface.registerHandle(barrett_vel_handle_j6);

   hardware_interface::JointHandle barrett_vel_handle_j7(jnt_state_interface.getHandle("finger_2_dist_joint"), &vel_cmd[6+mitsubishi_joint_number]);
   jnt_vel_interface.registerHandle(barrett_vel_handle_j7);

   hardware_interface::JointHandle barrett_vel_handle_j8(jnt_state_interface.getHandle("finger_3_dist_joint"), &vel_cmd[7+mitsubishi_joint_number]);
   jnt_vel_interface.registerHandle(barrett_vel_handle_j8);



   registerInterface(&jnt_vel_interface);

   // connect and register the effort interface
   hardware_interface::JointHandle barrett_eff_handle_j1(jnt_state_interface.getHandle("finger_1_med_joint"), &eff_cmd[0+mitsubishi_joint_number]);
   jnt_eff_interface.registerHandle(barrett_eff_handle_j1);

   hardware_interface::JointHandle barrett_eff_handle_j2(jnt_state_interface.getHandle("finger_2_med_joint"), &eff_cmd[1+mitsubishi_joint_number]);
   jnt_eff_interface.registerHandle(barrett_eff_handle_j2);

   hardware_interface::JointHandle barrett_eff_handle_j3(jnt_state_interface.getHandle("finger_3_med_joint"), &eff_cmd[2+mitsubishi_joint_number]);
   jnt_eff_interface.registerHandle(barrett_eff_handle_j3);

   hardware_interface::JointHandle barrett_eff_handle_j4(jnt_state_interface.getHandle("finger_1_prox_joint"), &eff_cmd[3+mitsubishi_joint_number]);
   jnt_eff_interface.registerHandle(barrett_eff_handle_j4);

   hardware_interface::JointHandle barrett_eff_handle_j5(jnt_state_interface.getHandle("finger_2_prox_joint"), &eff_cmd[4+mitsubishi_joint_number]);
   jnt_eff_interface.registerHandle(barrett_eff_handle_j5);

   hardware_interface::JointHandle barrett_eff_handle_j6(jnt_state_interface.getHandle("finger_1_dist_joint"), &eff_cmd[5+mitsubishi_joint_number]);
   jnt_eff_interface.registerHandle(barrett_eff_handle_j6);

   hardware_interface::JointHandle barrett_eff_handle_j7(jnt_state_interface.getHandle("finger_2_dist_joint"), &eff_cmd[6+mitsubishi_joint_number]);
   jnt_eff_interface.registerHandle(barrett_eff_handle_j7);

   hardware_interface::JointHandle barrett_eff_handle_j8(jnt_state_interface.getHandle("finger_3_dist_joint"), &eff_cmd[7+mitsubishi_joint_number]);
   jnt_eff_interface.registerHandle(barrett_eff_handle_j8);

   registerInterface(&jnt_eff_interface);


   // INIT MITSUBISHI
   // Open File Descriptor
   USB = open( "/dev/ttyUSB0",  O_RDWR | O_NOCTTY);

   // Error Handling
   if ( USB < 0 )
   {
       std::cout << "Error " << errno << " opening " << "/dev/ttyUSB0" << ": " << strerror (errno) << std::endl;
   }

   // Configure Port
   memset (&tty, 0, sizeof tty);

   // Error Handling
   if ( tcgetattr ( USB, &tty ) != 0 )
   {
       std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
   }


   // Set Baud Rate
   cfsetospeed (&tty, B19200);
   cfsetispeed (&tty, B19200);

   long BAUD    =B19200;

   tty.c_cflag = BAUD | CRTSCTS | CS8 | CLOCAL | CREAD | PARENB;

   tty.c_cc[VMIN]=0;
   tty.c_cc[VTIME]=20;

   // Flush Port, then applies attributes
   tcflush( USB, TCIFLUSH );

   if ( tcsetattr ( USB, TCSANOW, &tty ) != 0)
   {
       std::cout << "Error " << errno << " from tcsetattr" << std::endl;
   }
   else
       std::cout << "connected successfuly" <<std::endl;
   usleep(10000);

   char buf [256];
   read( USB, &buf, 1); // CLEAN BUFFER
   readMitsubishiHW();

   std::cout << "mitsubishi initialized" << std::endl;

   // INIT BARRETT//******************************************
   //*********************added new ***************************
   finger_1_cloud=pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);
   finger_2_cloud=pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);
   finger_3_cloud=pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);


   finger_1_cloud->points.resize(24);
   finger_2_cloud->points.resize(24);
   finger_3_cloud->points.resize(24);

   double x_offset=-0.021;
   double y_offset=-0.012;
   double z_offset=-0.005;


   double x_spacing=0.005;
   double z_spacing=0.005;
   for(int i=0; i<8; ++i)
   {
       for(int j=0; j<3; ++j)
       {
           finger_1_cloud->points[j+i*3].x=x_offset-i*x_spacing;
           finger_1_cloud->points[j+i*3].y=y_offset;
           finger_1_cloud->points[j+i*3].z=z_offset+j*z_spacing;

	   finger_2_cloud->points[j+i*3].x=x_offset-i*x_spacing;
           finger_2_cloud->points[j+i*3].y=y_offset;
           finger_2_cloud->points[j+i*3].z=z_offset+j*z_spacing;

           finger_3_cloud->points[j+i*3].x=x_offset-i*x_spacing;
           finger_3_cloud->points[j+i*3].y=y_offset;
           finger_3_cloud->points[j+i*3].z=z_offset+j*z_spacing;
       }
   }

   
   
   //************************************************************
   

   int BH_MODEL=280;
   wamdriver = boost::shared_ptr<OWD::WamDriver> (new OWD::WamDriver(canbus_number,BH_MODEL,forcetorque,tactile));
   std::cout << "calibration file name: "<<calibration_filename<< std::endl;
   try
   {
       if (! wamdriver->Init(calibration_filename.c_str()))
       {
           ROS_FATAL("WamDriver::Init() returned false; exiting.");
           exit(1);
       }
   }
   catch (int error)
   {
       ROS_FATAL("Error during WamDriver::Init(); exiting.");
       exit(1);
   }
   catch (const char *errmsg)
   {
       ROS_FATAL("Error during WamDriver::Init(): %s",errmsg);
       exit(1);
   }

    bhd=boost::shared_ptr<BHD_280>(new BHD_280(wamdriver->bus)); //added new *******

   //INITIALIZE
   readHW();

   prev_time=ros::Time::now();

   pos_cmd=pos;
   pos_cmd_previous=pos_cmd;
   std::fill(vel.begin(), vel.end(), 0.0);
   std::fill(eff.begin(), eff.end(), 0.0);
   std::fill(vel_cmd.begin(), vel_cmd.end(), 0.0);//added new *******
   std::fill(vel_cmd_previous.begin(), vel_cmd_previous.end(), 0.0);//added new *******
   std::fill(eff_cmd.begin(), eff_cmd.end(), 0.0);//added new *******
   std::fill(eff_cmd_previous.begin(), eff_cmd_previous.end(), 0.0);//added new *******
   
   //*******************added new ***************************************************
   tactile_msg.finger1.resize(24);
   tactile_msg.finger2.resize(24);
   tactile_msg.finger3.resize(24);
   tactile_msg.palm.resize(24);
   pub_tactile=n.advertise<owd_msgs::BHTactile>("tactile",10);
   pub_tactile_pcl=n.advertise<sensor_msgs::PointCloud2>("point_cloud",10);
   pub_tactile_finger_1_pcl=n.advertise<sensor_msgs::PointCloud2>("finger_1",10);
   pub_tactile_finger_2_pcl=n.advertise<sensor_msgs::PointCloud2>("finger_2",10);
   pub_tactile_finger_3_pcl=n.advertise<sensor_msgs::PointCloud2>("finger_3",10);

   //********************************************************************************
   
   // PRINT
   for(int i=0; i< pos.size(); ++i)
   {
       std::cout << pos_cmd[i] << std::endl;
   }

   std::cout << "Init done!" << '\n';

   //tactile_sensors.resize(4);
   /*for(int i=0; i<tactile_sensors.size();++i)
   {
       tactile_sensors[i].resize(24);
   }*/

   tactile_timer = n.createTimer(ros::Rate(tactile_pub_freq_).expectedCycleTime(), &MitsubishiBarrettHardwareInterface::tactilePublish, this);// added new ***************
  // tact=boost::shared_ptr<Tactile>(new Tactile(wamdriver->bus)); // commented new ************

   return;
}

MitsubishiBarrettHardwareInterface::~MitsubishiBarrettHardwareInterface()
{}

void MitsubishiBarrettHardwareInterface::readHW()
{
   readMitsubishiHW();
   readBarrettHW();
}

void MitsubishiBarrettHardwareInterface::writeHW()
{
   //writeBarrettHW();
   writeMitsubishiHW();
   //writeBarrettHW();
}


void MitsubishiBarrettHardwareInterface::readMitsubishiHW()
{
   // WRITE READ to robot
   unsigned char cmd_msg[] = "1\r\n";
   int n_written = 0;

   do
   {
       n_written += write( USB, &cmd_msg[n_written], 1 );
   }
   while (cmd_msg[n_written-1] != '\n');

   // READ RESPONSE (R)
   char buf [256];
   memset (&buf, '\0', sizeof buf);
   int n = 0;
   std::string response;

   do
   {
       n += read( USB, &buf, 1);
       response.append( buf );
   }
   while( buf[0] != '\n');

   if (response.find("R\r\n") == std::string::npos)
   {
       std::cout << "didn-t find R!" << '\n';
       exit(-1);
   }

   response.clear();
   // END READ

   // READ JOINTS STATE
   n_written = 0;
   memset (&buf, '\0', sizeof buf);

   do
   {
       n_written += read( USB, &buf, 1);
       response.append( buf );

   }
   while( buf[0] != '\n');

   std::stringstream convertor(response);

   // READ RESPONSE (E)
   memset (&buf, '\0', sizeof buf);
   n = 0;

   do
   {
       n += read( USB, &buf, 1);
       response.append( buf );
   }
   while( buf[0] != '\n');

   if (response.find("E\r\n") == std::string::npos)
   {
       std::cout << "didn-t find E!" << '\n';
       exit(-1);
   }
   // END READ

   char dummy_char;
   convertor >> dummy_char
             >> pos[0]
             >> dummy_char
             >> pos[1]
             >> dummy_char
             >> pos[2]
             >> dummy_char
             >> pos[3]
             >> dummy_char
             >> pos[4]
             >> dummy_char
             >> pos[5]
             >> dummy_char
             >> pos[6]
             >> dummy_char;

   // convert to radians and add to state
   for(int i=0; i< mitsubishi_joint_number; ++i)
   {
       pos[i]=pos[i]*(DEG_TO_RAD);
   }

   eff[0]=0.0;
   eff[1]=0.0;
   eff[2]=0.0;
   eff[3]=0.0;
   eff[4]=0.0;
   eff[5]=0.0;

   vel[0]=0.0;
   vel[1]=0.0;
   vel[2]=0.0;
   vel[3]=0.0;
   vel[4]=0.0;
   vel[5]=0.0;

//    ft_get_status = bus->ft_get_data(ft_values,ft_filtered_values);


}


void MitsubishiBarrettHardwareInterface::writeMitsubishiHW()
{

   bool new_command=false;
   for(int i=0; i<mitsubishi_joint_number;++i)
   {
       if(!isEqual(pos_cmd_previous[i],pos_cmd[i],0.00001))
       {
           new_command=true;
           break;
       }
   }
   if(!new_command)
   {
       //        vel_cmd_previous=vel_cmd;
       return;
   }
   static int new_command_count=0;
   new_command_count++;
   std::cout << "new command:"<< new_command_count << std::endl;
   //boost::mutex::scoped_lock lock(io_mutex);
   // WRITE MOVE to robot
   unsigned char cmd_msg[] = "2\r\n";
   int n_written = 0;

   do
   {
       n_written += write( USB, &cmd_msg[n_written], 1 );
   }
   while (cmd_msg[n_written-1] != '\n');

   // READ RESPONSE (M)
   char buf [256];
   memset (&buf, '\0', sizeof buf);
   int n = 0;
   std::string response;

   do
   {
       n += read( USB, &buf, 1);
       response.append( buf );
   }
   while( buf[0] != '\n');

   if (response.find("M\r\n") == std::string::npos)
   {
       std::cout << "didn-t find M!" << '\n';
       exit(-1);
   }

   response.clear();
   // END READ

   std::stringstream write_msg;
   //std::cout << cmd[1] << std::endl;
   write_msg << double(pos_cmd[0]) << "," <<pos_cmd[1] << "," << pos_cmd[2] << "," << pos_cmd[3] << "," << pos_cmd[4] << "," << pos_cmd[5] << "\r\n";

   std::string write_str=write_msg.str();
   //std::cout << "writing command:" << write_str<< std::endl;

   // Write command
   write( USB, write_str.c_str(), write_str.size());
   pos_cmd_previous=pos_cmd;
   // READ RESPONSE (E)
   memset (&buf, '\0', sizeof buf);
   n = 0;
   //std::cout << "getting respoinse"<< std::endl;
   
   //Barrett write HW
   std::vector<double> write_cmd;
   write_cmd.push_back(pos_cmd[0+mitsubishi_joint_number]);
   write_cmd.push_back(pos_cmd[1+mitsubishi_joint_number]);
   write_cmd.push_back(pos_cmd[2+mitsubishi_joint_number]);
   write_cmd.push_back(pos_cmd[3+mitsubishi_joint_number]);
   wamdriver->bus->hand_move(write_cmd);
   pos_cmd_previous=pos_cmd;
   
   
   do
   {
       n += read( USB, &buf, 1);
       response.append( buf );
   }
   while( buf[0] != '\n');

   if (response.find("E\r\n") == std::string::npos)
   {
       std::cout << "didn-t find E!" << '\n';
       exit(-1);
   }


   // END READ
}
//************************************************added new ********************************************************
bool MitsubishiBarrettHardwareInterface::ftPublish(const ros::TimerEvent& e)

{
    static double ft_values[6];
    static double ft_filtered_values[6];
    int ft_get_status;
    ft_get_status = wamdriver->bus->ft_get_data(ft_values,ft_filtered_values);
    
    //forcetorque_state message always publishes
    //when sensor is actively saturated, saturated_axes=255
    //When sensor has no issues, saturated_axes=0
    //When sensor has been saturated since latest re-tare:
    //  saturated_axes = integer of bitmap of saturated cells, 000001-111111
    ft_state.header.stamp = ros::Time::now();
    ft_state.saturated_axes = wamdriver->bus->ft_get_state();
    ft_state.wrench.force.x = ft_filtered_values[0];
    ft_state.wrench.force.y = ft_filtered_values[1];
    ft_state.wrench.force.z = ft_filtered_values[2];
    ft_state.wrench.torque.x = ft_filtered_values[3];
    ft_state.wrench.torque.y = ft_filtered_values[4];
    ft_state.wrench.torque.z = ft_filtered_values[5];
    pub_ft_state.publish(ft_state);
    
    if (ft_get_status != OW_SUCCESS)
    {
        ROS_DEBUG_NAMED("ft","Unable to get data from Force/Torque sensor");
        return false;
    }
    
    // set the time
    ft_vals.header.stamp = ros::Time::now();
    // fill in the raw values
    ft_vals.wrench.force.x=ft_values[0];
    ft_vals.wrench.force.y=ft_values[1];
    ft_vals.wrench.force.z=ft_values[2];
    ft_vals.wrench.torque.x=ft_values[3];
    ft_vals.wrench.torque.y=ft_values[4];
    ft_vals.wrench.torque.z=ft_values[5];
    pub_ft.publish(ft_vals);
    // overwrite with the filtered values
    ft_vals.wrench.force.x= ft_filtered_values[0];
    ft_vals.wrench.force.y= ft_filtered_values[1];
    ft_vals.wrench.force.z= ft_filtered_values[2];
    ft_vals.wrench.torque.x=ft_filtered_values[3];
    ft_vals.wrench.torque.y=ft_filtered_values[4];
    ft_vals.wrench.torque.z=ft_filtered_values[5];
    pub_filtered_ft.publish(ft_vals);
    if (wamdriver->bus->accelerometer_data)
    {
        accel_vals.x = wamdriver->bus->accelerometer_data[0];
        accel_vals.y = wamdriver->bus->accelerometer_data[1];
        accel_vals.z = wamdriver->bus->accelerometer_data[2];
        pub_accel.publish(accel_vals);
    }
    return true;
}

void MitsubishiBarrettHardwareInterface::tactilePublish(const ros::TimerEvent& e)
{
    if (wamdriver->bus->tactile_get_data(&tactile_msg.finger1[0],
                                         &tactile_msg.finger2[0],
                                         &tactile_msg.finger3[0],
                                         &tactile_msg.palm[0]) != OW_SUCCESS)
    {
        ROS_WARN_NAMED("tactile","Unable to get data from Tactile sensors");
        return;
    }

    pub_tactile.publish(tactile_msg);


    for(int f=0; f<4; ++f)
    {
        switch(f)
        {
        case 0:
            finger_1_cloud->header.frame_id="finger_1_dist_link";
            break;
        case 1:
            finger_2_cloud->header.frame_id="finger_2_dist_link";
            break;
        case 2:
            finger_3_cloud->header.frame_id="finger_3_dist_link";
            break;
        case 3:
            break;
        }

        for(int i=0; i<8; ++i)
        {
            for(int j=0; j<3; ++j)
            {
                switch(f)
                {
                case 0:
                    if(tactile_msg.finger1[j+i*3]<2.5)
                    {
                        finger_1_cloud->points[j+i*3].intensity=0.0;
                    }
                    else
                    {
                        finger_1_cloud->points[j+i*3].intensity=255*tactile_msg.finger1[j+i*3]/18.0;
                    }
                    break;
                case 1:
                    if(tactile_msg.finger2[j+i*3]<2.5)
                    {
                        finger_2_cloud->points[j+i*3].intensity=0.0;
                    }
                    else
                    {
                        finger_2_cloud->points[j+i*3].intensity=255*tactile_msg.finger2[j+i*3]/18.0;
                    }
                    break;
                case 2:
                    if(tactile_msg.finger3[j+i*3]<2.5)
                    {
                        finger_3_cloud->points[j+i*3].intensity=0.0;
                    }
                    else
                    {
                        finger_3_cloud->points[j+i*3].intensity=255*tactile_msg.finger3[j+i*3]/18.0;
                    }
                    break;
                case 3:
                    break;
                }
            }
        }
    }

    pub_tactile_finger_1_pcl.publish(*finger_1_cloud);
    pub_tactile_finger_2_pcl.publish(*finger_2_cloud);
    pub_tactile_finger_3_pcl.publish(*finger_3_cloud);

}
//***************************************************************************************************************

void MitsubishiBarrettHardwareInterface::readBarrettHW()
{
   readBarrettPositionAndComputeVelocity();
   readBarrettTorque();
}

void MitsubishiBarrettHardwareInterface::writeBarrettHW()
{
   // HOW TO HAVE SEVERAL INTERFACES AT THE SAME TIME?
   writeBarrettPosition();
   //writeBarrettVelocity();
}

void MitsubishiBarrettHardwareInterface::readBarrettPosition()
{
   wamdriver->bus->hand_get_positions(pos[0+mitsubishi_joint_number],
                                      pos[1+mitsubishi_joint_number],
                                      pos[2+mitsubishi_joint_number],
                                      pos[3+mitsubishi_joint_number]);
   pos[4+mitsubishi_joint_number]=pos[3+mitsubishi_joint_number];

   pos[5+mitsubishi_joint_number]=(pos[0+mitsubishi_joint_number])/2.4+0.6;
   pos[6+mitsubishi_joint_number]=(pos[1+mitsubishi_joint_number])/2.4+0.6;
   pos[7+mitsubishi_joint_number]=(pos[2+mitsubishi_joint_number])/2.4+0.6;

   //wamdriver->bus->hand_get_outer_links(pos[5],pos[6],pos[7]);
}

void MitsubishiBarrettHardwareInterface::readBarrettPositionAndComputeVelocity()
{
   ros::Time curr_time=ros::Time::now();
   ros::Duration period = curr_time-prev_time;
   std::vector<double> prev_pos=pos;

   for(int i=mitsubishi_joint_number;i<joint_number;++i)
   {
     //  std::cout << "previous_pos: "<<i <<" " <<pos[i]<<std::endl;
   }

   readBarrettPosition();
  // std::cout << period << std::endl;
   for(int i=mitsubishi_joint_number;i<joint_number;++i)
   {
      // std::cout << "current_pos: "<<i <<" " <<pos[i]<<std::endl;
       vel[i]=(pos[i]-prev_pos[i])/period.toSec();
   }
   prev_time=curr_time;
}

void MitsubishiBarrettHardwareInterface::readBarrettTorque()
{
   //std::cout << "read torque:" << std::endl;

   //static double ft_values[6];
   //static double ft_filtered_values[6];
   //int ft_get_status;
   //ft_get_status = wamdriver->bus->ft_get_data(ft_values,ft_filtered_values);
   //std::cout << "ft_get_status:" << ft_get_status<< std::endl;
   //std::cout << "ft_values:"<<ft_values[0]<< std::endl;

   wamdriver->bus->hand_get_strain(eff[0], eff[1], eff[2]);
}

void MitsubishiBarrettHardwareInterface::writeBarrettPosition()
{
   bool new_command=false;
   for(int i=mitsubishi_joint_number; i<joint_number;++i)
   {
       if(!isEqual(pos_cmd_previous[i],pos_cmd[i],0.00001))
       {
           new_command=true;
           break;
       }
   }
   if(!new_command)
   {
       //pos_cmd_previous=pos_cmd;
       return;
   }

   std::vector<double> write_cmd;
   write_cmd.push_back(pos_cmd[0+mitsubishi_joint_number]);
   write_cmd.push_back(pos_cmd[1+mitsubishi_joint_number]);
   write_cmd.push_back(pos_cmd[2+mitsubishi_joint_number]);
   write_cmd.push_back(pos_cmd[3+mitsubishi_joint_number]);
   wamdriver->bus->hand_move(write_cmd);
   pos_cmd_previous=pos_cmd;
}

void MitsubishiBarrettHardwareInterface::writeBarrettVelocity()
{
   bool new_command=false;
   for(int i=mitsubishi_joint_number; i<joint_number;++i)
   {
       if(!isEqual(vel_cmd_previous[i],vel_cmd[i],0.00001))
       {
           new_command=true;
           break;
       }
   }
   if(!new_command)
   {
       //        vel_cmd_previous=vel_cmd;
       return;
   }

   std::vector<double> write_cmd;
   write_cmd.push_back(vel_cmd[0+mitsubishi_joint_number]);
   write_cmd.push_back(vel_cmd[1+mitsubishi_joint_number]);
   write_cmd.push_back(vel_cmd[2+mitsubishi_joint_number]);
   write_cmd.push_back(vel_cmd[3+mitsubishi_joint_number]);
   wamdriver->bus->hand_velocity(write_cmd);
   vel_cmd_previous=vel_cmd;
}

void MitsubishiBarrettHardwareInterface::writeBarrettEffort()
{
   std::vector<double> write_cmd;
   write_cmd.push_back(eff_cmd[0]);
   write_cmd.push_back(eff_cmd[1]);
   write_cmd.push_back(eff_cmd[2]);
   write_cmd.push_back(eff_cmd[3]);
   wamdriver->bus->hand_torque(write_cmd);
}





// MitsubishiBarrettHardwareInterface::MitsubishiBarrettHardwareInterface(const int & canbus_number_, bool & forcetorque_, bool & tactile_,  std::string & calibration_filename_, double & ft_pub_freq_, double & tactile_pub_freq_) :
//     n("~"),
//     barrett_arm_interface(canbus_number_,
//                           forcetorque_,
//                           tactile_,
//                           calibration_filename_,
//                           ft_pub_freq_,
//                           tactile_pub_freq_)
// {
//     /*pos.resize(joint_number);
//     vel.resize(joint_number);
//     eff.resize(joint_number);
//     pos_cmd.resize(joint_number);
//     vel_cmd.resize(joint_number);
//     eff_cmd.resize(joint_number);
//     pos_cmd_previous.resize(joint_number);
//     vel_cmd_previous.resize(joint_number);
//     eff_cmd_previous.resize(joint_number);*/
//     std::string port = "/dev/ttyUSB0";
//     MitsubishiArmInterface mitsubishi_arm_interface(port);
//     
//     mitsubishi_arm_interface.init(jnt_state_interface,
//                                   jnt_pos_interface);
// 
//     barrett_arm_interface.init(jnt_state_interface,
//                                jnt_pos_interface,
//                                jnt_vel_interface,
//                                jnt_eff_interface);
// 
//     registerInterface(&jnt_state_interface);
//     registerInterface(&jnt_pos_interface);
//     registerInterface(&jnt_vel_interface);
//     registerInterface(&jnt_eff_interface);
// 
//     return;
// }






