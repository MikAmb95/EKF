#include "ros/ros.h"
#include "boost/thread.hpp"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/mman.h>
#include <sys/io.h>
#include <sys/time.h>
#include <netdb.h>

#include <chrono>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chaindynparam.hpp>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>


using namespace Eigen;
using namespace std;
using namespace KDL;
using namespace std::chrono;



int row = 30001;
int n = row;
int col = 7;
float vett[30001][7];
float q1[30001];
float q2[30001];
float q3[30001];
float q4[30001];
float q5[30001];
float q6[30001];
float q7[30001];                                                                                                                                 
                                                                                                                                




typedef struct ROBOT_STATE {
  double jstate[7];
}ROBOT_STATE;

typedef struct ROBOT_CMD {
  double jcmd[7];
}ROBOT_CMD;

//Creazione socket in LETTURA
inline bool listener_socket(int port_number, int *sock) {
      sockaddr_in si_me;

  if ( (*sock=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
    std::cout << "Listener::Open: error during socket creation!" << std::endl;
    return false;
  }

  memset((char *) &si_me, 0, sizeof(si_me));

  /* allow connections to any address port */
  si_me.sin_family = AF_INET;
  si_me.sin_port = htons(port_number);
  si_me.sin_addr.s_addr = htonl(INADDR_ANY);
  int bind_ok = bind(*sock, (struct sockaddr*)&si_me, sizeof(si_me));

  if ( bind_ok == -1 )
    return false;
  else
    return true;

}

//Creazione socket in SCRITTURA
inline int create_socket(char* dest, int port, int *sock) {
  struct sockaddr_in temp;
  struct hostent *h;
  int error;

  temp.sin_family=AF_INET;
  temp.sin_port=htons(port);
  h=gethostbyname(dest);

  if (h==0) {
    printf("Gethostbyname fallito\n");
    exit(1);
  }

  bcopy(h->h_addr,&temp.sin_addr,h->h_length);
  *sock=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  error=connect(*sock, (struct sockaddr*) &temp, sizeof(temp));
  return error;
}




class KUKA_INVKIN {
	public:
		KUKA_INVKIN();
		void run();
		bool init_robot_model();
                void pd_g();


                void joint_states_cb(std_msgs::Float64 js);
                void joint_states_cb1(std_msgs::Float64 js);
                void joint_states_cb2(std_msgs::Float64 js);
                void joint_states_cb3(std_msgs::Float64 js);
                void joint_states_cb4(std_msgs::Float64 js);
                void joint_states_cb5(std_msgs::Float64 js);
                void joint_states_cb6(std_msgs::Float64 js);


		void ctrl_loop();
                VectorXd RG_function();
                VectorXd forw_dy2(VectorXd pos,VectorXd vel,VectorXd v);

	private:
      	        int _jstate_socket;
                int _jcommand_socket;

	        
                ros::Subscriber _js_sub;
                ros::Subscriber _js_sub1;
                ros::Subscriber _js_sub2;
                ros::Subscriber _js_sub3;
                ros::Subscriber _js_sub4;
                ros::Subscriber _js_sub5;
                ros::Subscriber _js_sub6;

                _Float64 position[7];
 
		ros::NodeHandle _nh;


		KDL::Chain _k_chain;

                Eigen::MatrixXd Kd; 	
		Eigen::MatrixXd Kp;
	

                bool first_flag;

	
};


bool KUKA_INVKIN::init_robot_model() {
	std::string robot_desc_string;
	_nh.param("robot_description", robot_desc_string, std::string());
	

        Kp = MatrixXd::Zero(7,7);
        Kp(0,0) = 500;
        Kp(1,1) = 500;
        Kp(2,2) = 500;
        Kp(3,3) = 700;
        Kp(4,4) = 500;
        Kp(5,5) = 500;
        Kp(6,6) = 500;


	Kd = MatrixXd::Zero(7,7);
        Kd(0,0) = 10;
        Kd(1,1) = 10;
        Kd(2,2) = 10;
        Kd(3,3) = 10;
        Kd(4,4) = 10;
        Kd(5,5) = 10;
        Kd(6,6) = 10;
	return true;
}


//SUBSCRIBER

void KUKA_INVKIN::joint_states_cb( std_msgs::Float64 js ) {

int x = 0;

	position[0] = js.data;
        //cout<<" 1: "<<position[0]<<endl;
        //cin>>x;
        
}

void KUKA_INVKIN::joint_states_cb1( std_msgs::Float64 js ) {

	position[1] = js.data;
       //cout<<" 2: "<<position[1]<<endl;
}

void KUKA_INVKIN::joint_states_cb2( std_msgs::Float64 js ) {

	position[2] = js.data;
        //cout<<" 3: "<<position[2]<<endl;
}

void KUKA_INVKIN::joint_states_cb3( std_msgs::Float64 js ) {

	position[3] = js.data;
        //cout<<" 4: "<<position[3]<<endl;
        
}

void KUKA_INVKIN::joint_states_cb4( std_msgs::Float64 js ) {

	position[4] = js.data;
        //cout<<" 5: "<<position[4]<<endl;
}

void KUKA_INVKIN::joint_states_cb5( std_msgs::Float64 js ) {

	position[5] = js.data;
       //cout<<" 6: "<<position[5]<<endl;
}

void KUKA_INVKIN::joint_states_cb6( std_msgs::Float64 js ) {

	position[6] = js.data;
        //cout<<" 7: "<<position[6]<<endl;
        }


// CODE FROM HERE

KUKA_INVKIN::KUKA_INVKIN() {

	if (!init_robot_model()) exit(1); 
	

        _js_sub = _nh.subscribe("/q1_des", 0, &KUKA_INVKIN::joint_states_cb, this);
        _js_sub1 = _nh.subscribe("/q2_des", 0, &KUKA_INVKIN::joint_states_cb1, this);
        _js_sub2 = _nh.subscribe("/q3_des", 0, &KUKA_INVKIN::joint_states_cb2, this);
        _js_sub3 = _nh.subscribe("/q4_des", 0, &KUKA_INVKIN::joint_states_cb3, this);
        _js_sub4 = _nh.subscribe("/q5_des", 0, &KUKA_INVKIN::joint_states_cb4, this);
        _js_sub5 = _nh.subscribe("/q6_des", 0, &KUKA_INVKIN::joint_states_cb5, this);
        _js_sub6 = _nh.subscribe("/q7_des", 0, &KUKA_INVKIN::joint_states_cb6, this);



	

	listener_socket(9030, &_jstate_socket);
        create_socket("192.170.10.146",9031,&_jcommand_socket);

}


void KUKA_INVKIN::pd_g() {

int slen2, rlen2;	
ROBOT_STATE rs2;
sockaddr_in si_other2;
ROBOT_CMD rc;

VectorXd q_in = VectorXd::Zero(7);
VectorXd q_0 = VectorXd::Zero(7);
VectorXd first_q_0 = VectorXd::Zero(7);
VectorXd q_dot = VectorXd::Zero(7);
VectorXd q_temp = VectorXd::Zero(7);

VectorXd vv = VectorXd::Zero(7);
VectorXd pq = VectorXd::Zero(7);
VectorXd vq = VectorXd::Zero(7);

int x;

float Ts;
float K;


        ofstream myfile;
        ofstream myfile2;
        myfile.open("/home/utente/ros_ws/src/iiwa_kdl/src/data_robot.m");
        myfile2.open("/home/utente/ros_ws/src/iiwa_kdl/src/sensor_robot.m");
        //myfile_final.open("/home/utente/ros_ws/src/iiwa_kdl/src/trans_matrix_final.csv");
        myfile<<"q = [";
        myfile2<<"q2 = [";


/*
rlen2 = recvfrom( _jstate_socket, &rs2, sizeof(rs2),0,(struct sockaddr*)&si_other2, (socklen_t*)&slen2);
          if(rlen2>0) {
          cout<<"inside"<<endl;
          for(int i=0; i<7; i++) {
          q_0(i) = rs2.jstate[i];
          first_q_0(i) = q_0(i);
         }
}

*/
cout<<"vv: "<<vv.transpose()<<endl;
cout<<"q0: "<<q_0.transpose()<<endl;
cout<<"fq0: "<<first_q_0.transpose()<<endl;



q_0(0) = 90*3.14/180;
q_0(1) = -7*3.14/180;
q_0(2) = -13*3.14/180;
q_0(3) = 108*3.14/180;
q_0(4) = -90*3.14/180;
q_0(5) = -90*3.14/180;
q_0(6) = 116*3.14/180;

for(int i=0; i<7; i++) {
          first_q_0(i) = q_0(i);
         }



for(int i=0; i<7; i++) {
       vv(i) = first_q_0(i)+position[i];
   }


cout<<"vv: "<<vv.transpose()<<endl;
cout<<"q0: "<<q_0.transpose()<<endl;
cout<<"fq0: "<<first_q_0.transpose()<<endl;

double my_time = 0;
auto start = high_resolution_clock::now();


//ros::Rate r(1000);
       while( true) {
       auto start = high_resolution_clock::now();
       
                
       rlen2 = recvfrom( _jstate_socket, &rs2, sizeof(rs2),0,(struct sockaddr*)&si_other2, (socklen_t*)&slen2);
          if(rlen2>0) {
                 cout<<"Receive"<<endl;
                 for(int i=0; i<7; i++) 
                 q_in(i) = rs2.jstate[i];
        }
            else {
            cout<<"Not Receive"<<endl;
            }


        Ts = 0.003;
        K = 1000;
        //q_temp = (q_in - q_0)/0.003;
        q_dot = (2*K*q_in - 2*K*q_0 - (Ts*K-2)*q_dot)/(Ts*K+2);
        q_0 = q_in;

          


        
        for(int i=0; i<7; i++) {
        vv(i) = first_q_0(i)+position[i];
        }

        
        //vq = vv;
        

        


        //Eigen::VectorXd e =  _ref_q->data - q_in + q0->data;
        Eigen::VectorXd e = vv - q_in; 
        //cout<<"ERR: "<<e.transpose()<<endl;
        Eigen::VectorXd de = q_dot; 
        //cout<<"ERR: "<<de.transpose()<<endl;
        
     
        Eigen::VectorXd tao = -Kd*de + Kp*e;
        
        for (int i=0; i<7; i++){
                rc.jcmd[i] = tao(i);
                
                }
                
                
        write( _jcommand_socket, &rc, sizeof(rc) ); //write commands over socket
        
        if(my_time<80.0){
        myfile<<vv.transpose()<<" ";
        myfile<<"\n";
        myfile2<<q_in.transpose()<<" ";
        myfile2<<"\n";
        }
        else{
        myfile<<vv.transpose()<<"]; \n";
        myfile.close();
        myfile2<<q_in.transpose()<<"]; \n";
        myfile2.close();
        } 

        auto stop = high_resolution_clock::now();
     
                
        auto duration = duration_cast<microseconds>(stop - start);
        my_time = duration.count()*0.000001;
        cout<<"time: "<<my_time<<endl;
                
        //r.sleep();
      
        }




}


void KUKA_INVKIN::run() {

	boost::thread pd_g ( &KUKA_INVKIN::pd_g, this);
	
	ros::spin();	

}




int main(int argc, char** argv) {
      

        cout<<"INIT"<<endl;

        
	ros::init(argc, argv, "pd_ral");
	KUKA_INVKIN ik;
	ik.run();

	return 0;
}
