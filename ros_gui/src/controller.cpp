/**
 * @file /src/qnode.cpp
 *
 * @brief Controller central!
 *
 * @date March 2015
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
#include <QtCore>
#include <ros/ros.h>
#include <ros/network.h>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/ros_gui/controller.hpp"

#include <iostream>
#include <opencv2/core/core.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ros_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

//Para matrizes de ponderação: Q 100*I e R = I
const float Controller::Kdlqr[3][6] = {
    { 0.0000,   -7.3404,    0.1982,    0.0000,   -5.0455,    0.3356},
    { 6.3570,    3.6702,    0.1982,    4.3695,    2.5227,    0.3356},
    {-6.3570,    3.6702,    0.1982,   -4.3695,    2.5227,    0.3356}
                           };

/*Referentes ao preditor Otimo*/
const float Controller::Ag[6][6] = {
    {0.8218,         0,         0,         0,         0,         0},
    {     0,    0.8218,         0,         0,         0,         0},
    {     0,         0,    0.5888,         0,         0,         0},
    {0.8218,         0,         0,    1.0000,         0,         0},
    {     0,    0.8218,         0,         0,    1.0000,         0},
    {     0,         0,    0.5888,         0,         0,    1.0000}
                        };

const float Controller::Bg[6][3] = {
    {      0,    0.0400,   -0.0400},
    {-0.0461,    0.0231,    0.0231},
    { 0.9868,    0.9868,    0.9868},
    {      0,    0.0400,   -0.0400},
    {-0.0461,    0.0231,    0.0231},
    { 0.9868,    0.9868,    0.9868}
                        };

/*Referentes ao preditor de Smith*/
const float Controller::Ad[3][3] = {
    {0.8218,         0,         0},
    {     0,    0.8218,         0},
    {     0,         0,    0.5888}
                        };

const float Controller::Bd[3][3] = {
    {        0,    0.0400,   -0.0400},
    {    -0.0461,    0.0231,    0.0231},
    {    0.9868,    0.9868,    0.9868}
                        };

const float Controller::Cd[3][3] = {
                         {1, 0, 0},
                         {0, 1, 0},
                         {0, 0, 1}
                        };

 Mat AA;
 Mat BB;
 Mat CC;

 Mat Q;
 Mat R;

 Mat W;

 float TrajX[100];             //vetor contendo a trajetória em X
 float TrajY[100];             //vetor contendo a trajetória em Y
 float TrajTheta[100];
 int Index;
 int TrajPoints;
 int TrajOK;

 float xr,yr,theta;

 float Vref[3]; //Ainda é utilizado devido ao log
 float dU[3], U[3];
 float vel[3], old_vel[3];
 float velrad[3];
 float x[3], old_x[3];
 float xFiltrado[3];
 float velm[3], new_velm[3];
 float difvel[3];

 char PWM[4];

Controller::Controller(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

Controller::~Controller() {
	if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
	}
	wait();
}



bool Controller::init(SerialPort* device) {
	ros::init(init_argc,init_argv,"ros_gui_controller");
	if ( ! ros::master::check() ) {
		return false;
	}

	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;

  dataReceived = false;
  enable = false;

  /*-----------------init matrices----------------*/

  // Matrizes Aumentadas
  cv::Mat Aa = (cv::Mat_<float>(6,6) << 0.8218, 0, 0, 0, 0, 0,
                                        0, 0.8218, 0, 0, 0, 0,
                                        0, 0, 0.5888, 0, 0, 0,
                                        0.8218, 0, 0, 1, 0, 0,
                                        0, 0.8218, 0, 0, 1, 0,
                                        0, 0, 0.5888, 0, 0, 1);

  cv::Mat Ba = (cv::Mat_<float>(6,3) <<   0,    0.0400,   -0.0400,
                                          -0.0461,    0.0231,    0.0231,
                                          0.9868,    0.9868,    0.9868,
                                          0,    0.0400,   -0.0400,
                                          -0.0461,    0.0231,    0.0231,
                                          0.9868,    0.9868,    0.9868);

  //Produzindo as Matrizes Caligraficas
  AA.create(0,0,CV_32F);
  BB.create(0,0,CV_32F);
  CC = Mat::eye(3*HP, 6*HP, CV_32F);

  Q = Mat::eye(3*HP, 3*HP, CV_32F)*25;
  R = Mat::eye(3*HC, 3*HC, CV_32F);

  Mat aux_Ba;
  Mat aux_Aa;

  aux_Ba.create(0,0,CV_32F);
  aux_Aa.create(0,0,CV_32F);

  aux_Ba.push_back(Ba);
  aux_Aa.push_back(Aa);

  Mat aux_BB = Ba;
  AA.push_back(Aa);

  for(int i = 1; i < HP; i++) {
      aux_Ba = aux_Aa*Ba;
      aux_Aa = aux_Aa*Aa;

      AA.push_back(aux_Aa);
      aux_BB.push_back(aux_Ba);
  }

  for(int i = 1; i < HC; i++) {
      aux_Ba = Mat::zeros(Ba.rows*i,Ba.cols,CV_32F);
      aux_Ba.push_back(aux_BB.colRange(0,Ba.cols).rowRange(0,Ba.rows*(HP-i)));
      hconcat(aux_BB,aux_Ba,aux_BB);
  }

  BB.push_back(aux_BB);

  aux_Aa.release();
  aux_Ba.release();
  aux_BB.release();

  /*-----------------init vectors-----------------*/

  vel[0] = vel[1] = vel[2] = 0;

  xFiltrado[0] = xFiltrado[1] = xFiltrado[2] = 0;

  velm[0] = velm[1] = velm[2] = 0;

  x[0] = x[1] = x[2] = 0;

  dU[0] = dU[1] = dU[2] = 0;
  U[0] = U[1] = U[2] = 0;

  PWM[0] = PWM[1] = PWM[2] = PWM[3] = 0;

  /*-----------------init trajectory-----------------*/

  TrajOK = 0;
  Index = 0;
  TrajPoints = 68;

  //Combined:
      TrajX[0] = 0;
      TrajX[1] = 0.1000;
      TrajX[2] = 0.2000;
      TrajX[3] = 0.3000;
      TrajX[4] = 0.4000;
      TrajX[5] = 0.5000;
      TrajX[6] = 0.6000;
      TrajX[7] = 0.7000;
      TrajX[8] = 0.8000;
      TrajX[9] = 0.9000;
      TrajX[10] = 1.0000;
      TrajX[11] = 1.1000;
      TrajX[12] = 1.2000;
      TrajX[13] = 1.3000;
      TrajX[14] = 1.4000;
      TrajX[15] = 1.5000;
      TrajX[16] = 1.6000;
      TrajX[17] = 1.6000;
      TrajX[18] = 1.6000;
      TrajX[19] = 1.6000;
      TrajX[20] = 1.6000;
      TrajX[21] = 1.6000;
      TrajX[22] = 1.6000;
      TrajX[23] = 1.6000;
      TrajX[24] = 1.6000;
      TrajX[25] = 1.6000;
      TrajX[26] = 1.6000;
      TrajX[27] = 1.7000;
      TrajX[28] = 1.8000;
      TrajX[29] = 1.9000;
      TrajX[30] = 2.0000;
      TrajX[31] = 2.1000;
      TrajX[32] = 2.1000;
      TrajX[33] = 2.1000;
      TrajX[34] = 2.1000;
      TrajX[35] = 2.1000;
      TrajX[36] = 2.1000;
      TrajX[37] = 2.1000;
      TrajX[38] = 2.1782;
      TrajX[39] = 2.2545;
      TrajX[40] = 2.3270;
      TrajX[41] = 2.3939;
      TrajX[42] = 2.4536;
      TrajX[43] = 2.5045;
      TrajX[44] = 2.5455;
      TrajX[45] = 2.5755;
      TrajX[46] = 2.5938;
      TrajX[47] = 2.6000;
      TrajX[48] = 2.5938;
      TrajX[49] = 2.5755;
      TrajX[50] = 2.5455;
      TrajX[51] = 2.5045;
      TrajX[52] = 2.4536;
      TrajX[53] = 2.3939;
      TrajX[54] = 2.3270;
      TrajX[55] = 2.2545;
      TrajX[56] = 2.1782;
      TrajX[57] = 2.1000;
      TrajX[58] = 2.0218;
      TrajX[59] = 1.9455;
      TrajX[60] = 1.8730;
      TrajX[61] = 1.8061;
      TrajX[62] = 1.7464;
      TrajX[63] = 1.6955;
      TrajX[64] = 1.6545;
      TrajX[65] = 1.6245;
      TrajX[66] = 1.6062;
      TrajX[67] = 1.6000;

      TrajY[0] = 0;
      TrajY[1] = 0.1000;
      TrajY[2] = 0.2000;
      TrajY[3] = 0.3000;
      TrajY[4] = 0.4000;
      TrajY[5] = 0.5000;
      TrajY[6] = 0.6000;
      TrajY[7] = 0.7000;
      TrajY[8] = 0.8000;
      TrajY[9] = 0.9000;
      TrajY[10] = 1.0000;
      TrajY[11] = 1.0000;
      TrajY[12] = 1.0000;
      TrajY[13] = 1.0000;
      TrajY[14] = 1.0000;
      TrajY[15] = 1.0000;
      TrajY[16] = 1.0000;
      TrajY[17] = 1.1000;
      TrajY[18] = 1.2000;
      TrajY[19] = 1.3000;
      TrajY[20] = 1.4000;
      TrajY[21] = 1.5000;
      TrajY[22] = 1.6000;
      TrajY[23] = 1.7000;
      TrajY[24] = 1.8000;
      TrajY[25] = 1.9000;
      TrajY[26] = 2.0000;
      TrajY[27] = 2.0000;
      TrajY[28] = 2.0000;
      TrajY[29] = 2.0000;
      TrajY[30] = 2.0000;
      TrajY[31] = 2.0000;
      TrajY[32] = 1.9000;
      TrajY[33] = 1.8000;
      TrajY[34] = 1.7000;
      TrajY[35] = 1.6000;
      TrajY[36] = 1.5000;
      TrajY[37] = 1.5000;
      TrajY[38] = 1.4938;
      TrajY[39] = 1.4755;
      TrajY[40] = 1.4455;
      TrajY[41] = 1.4045;
      TrajY[42] = 1.3536;
      TrajY[43] = 1.2939;
      TrajY[44] = 1.2270;
      TrajY[45] = 1.1545;
      TrajY[46] = 1.0782;
      TrajY[47] = 1.0000;
      TrajY[48] = 0.9218;
      TrajY[49] = 0.8455;
      TrajY[50] = 0.7730;
      TrajY[51] = 0.7061;
      TrajY[52] = 0.6464;
      TrajY[53] = 0.5955;
      TrajY[54] = 0.5545;
      TrajY[55] = 0.5245;
      TrajY[56] = 0.5062;
      TrajY[57] = 0.5000;
      TrajY[58] = 0.5062;
      TrajY[59] = 0.5245;
      TrajY[60] = 0.5545;
      TrajY[61] = 0.5955;
      TrajY[62] = 0.6464;
      TrajY[63] = 0.7061;
      TrajY[64] = 0.7730;
      TrajY[65] = 0.8455;
      TrajY[66] = 0.9218;
      TrajY[67] = 1.0000;

      TrajTheta[0] = 0;
      TrajTheta[1] = 0.7854;
      TrajTheta[2] = 0.7854;
      TrajTheta[3] = 0.7854;
      TrajTheta[4] = 0.7854;
      TrajTheta[5] = 0.7854;
      TrajTheta[6] = 0.7854;
      TrajTheta[7] = 0.7854;
      TrajTheta[8] = 0.7854;
      TrajTheta[9] = 0.7854;
      TrajTheta[10] = 0.7854;
      TrajTheta[11] = 0;
      TrajTheta[12] = 0;
      TrajTheta[13] = 0;
      TrajTheta[14] = 0;
      TrajTheta[15] = 0;
      TrajTheta[16] = 0;
      TrajTheta[17] = 1.5708;
      TrajTheta[18] = 1.5708;
      TrajTheta[19] = 1.5708;
      TrajTheta[20] = 1.5708;
      TrajTheta[21] = 1.5708;
      TrajTheta[22] = 1.5708;
      TrajTheta[23] = 1.5708;
      TrajTheta[24] = 1.5708;
      TrajTheta[25] = 1.5708;
      TrajTheta[26] = 1.5708;
      TrajTheta[27] = 0;
      TrajTheta[28] = 0;
      TrajTheta[29] = 0;
      TrajTheta[30] = 0;
      TrajTheta[31] = 0;
      TrajTheta[32] = 4.7124;
      TrajTheta[33] = 4.7124;
      TrajTheta[34] = 4.7124;
      TrajTheta[35] = 4.7124;
      TrajTheta[36] = 4.7124;
      TrajTheta[37] = 2*PI;
      TrajTheta[38] = 2*PI-0.1571;
      TrajTheta[39] = 2*PI-0.3142;
      TrajTheta[40] = 2*PI-0.4712;
      TrajTheta[41] = 2*PI-0.6283;
      TrajTheta[42] = 2*PI-0.7854;
      TrajTheta[43] = 2*PI-0.9425;
      TrajTheta[44] = 2*PI-1.0996;
      TrajTheta[45] = 2*PI-1.2566;
      TrajTheta[46] = 2*PI-1.4137;
      TrajTheta[47] = 2*PI-1.5708;
      TrajTheta[48] = 2*PI-1.7279;
      TrajTheta[49] = 2*PI-1.8850;
      TrajTheta[50] = 2*PI-2.0420;
      TrajTheta[51] = 2*PI-2.1991;
      TrajTheta[52] = 2*PI-2.3562;
      TrajTheta[53] = 2*PI-2.5133;
      TrajTheta[54] = 2*PI-2.6704;
      TrajTheta[55] = 2*PI-2.8274;
      TrajTheta[56] = 2*PI-2.9845;
      TrajTheta[57] = 2*PI-3.1416;
      TrajTheta[58] = 2*PI-3.2987;
      TrajTheta[59] = 2*PI-3.4558;
      TrajTheta[60] = 2*PI-3.6128;
      TrajTheta[61] = 2*PI-3.7699;
      TrajTheta[62] = 2*PI-3.9270;
      TrajTheta[63] = 2*PI-4.0841;
      TrajTheta[64] = 2*PI-4.2412;
      TrajTheta[65] = 2*PI-4.3982;
      TrajTheta[66] = 2*PI-4.5553;
      TrajTheta[67] = 2*PI-4.7124;

  /*------------------------------------------------*/

  xr = yr = theta = 0;

  Vref[0] = Vref[1] = Vref[2] = 0;

  // Add your ros communications here.
  start();

	port = device;
	std::cout << "Controller started\n";

	return true;
}

void Controller::run()
{

  /*--------------------------------*/

  float aux;

  ros::Rate loop_rate(16.667);

  QTime t1,t2;
  t1.start();
  t2.start();

	while ( ros::ok() )
	{

        if(!enable)
        {
            port->clearSerialData(0);
            t1.restart();
            t2.restart();

        }
        else {

            //Aqui chamar função para leitura serial
            if(readSerial()) {
                publisher(t1.restart());
            }

            old_vel[0] = vel[0];
            old_vel[1] = vel[1];
            old_vel[2] = vel[2];

            //O robô envia RPM *10 --> dividindo por 10 e dividindo por 60 para converter em voltas por seg
            //Depois multiplica por 2*PI para achar em rad/s
            //Velocidade escalar das rodas
            velrad[0] = (pkg_data.m1_velocity)*0.01047197;
            velrad[1] = (pkg_data.m2_velocity)*0.01047197;
            velrad[2] = (pkg_data.m3_velocity)*0.01047197;

            //Aqui converter para o sistema de coordenadas do robô
            //vel[0] = 0*vel[0] + 0.5774*vel[1] - 0.5774*vel[2];
            //vel[1] = -0.6667*vel[0] + 0.3333*vel[1] + 0.3333*vel[2];
            //vel[2] = 3.3333*vel[0] + 3.3333*vel[1] + 3.3333*vel[2];
            //Multiplica a matriz acima comentada por 0,0505
            //para obter a vel linear a partir de velrad
            vel[0] = 0*velrad[0] + 0.0292*velrad[1] - 0.0292*velrad[2];
            vel[1] = -0.0337*velrad[0] + 0.0168*velrad[1] + 0.0168*velrad[2];
            vel[2] = 0.1683*velrad[0] + 0.1683*velrad[1] + 0.1683*velrad[2];

            //Controlador Remoto
            if(true) {

                trajetoria(t2.restart());

                //'Chama' cotrolador
                //optmum_predictor();
                smith_predictor();

                U[0] = dU[0] + U[0];
                U[1] = dU[1] + U[1];
                U[2] = dU[2] + U[2];

                //Convertendo para PWM
                PWM[3] = 0;

               //Aqui eh preciso converter de tensão para PWM
               aux = 255-(255/6)*(6-U[0]);
                if(aux<0) {
                    aux = aux*(-1);
                    PWM[3] += 1;
                }
                PWM[0] = (unsigned char) aux;

                aux = 255-(255/6)*(6-U[1]);
                if(aux<0) {
                    aux = aux*(-1);
                    PWM[3] += 2;
                }
                PWM[1] = (unsigned char) aux;

                aux = 255-(255/6)*(6-U[2]);
                if(aux<0) {
                    aux = aux*(-1);
                    PWM[3] += 4;
                }
                PWM[2] = (unsigned char) aux;

                sC.serialSend("13A20040B09872", 2, PWM, 4, port->getFd());

            }
            //Controlador Embarcado
            else {

                trajetoria(t2.restart());

                char a[7], b;

                int v,vn,w;

                b = 0;

                v = (trunc(100*Vref[0]));
                vn = (trunc(100*Vref[1]));
                w = (trunc(100*Vref[2]));

                if(v<0) {
                    v = -1*v;
                    b +=1;
                }

                if(vn<0) {
                    vn = -1*vn;
                    b +=2;
                }

                if(w<0) {
                    w = -1*w;
                    b +=4;
                }

                a[0] = (char) ((0xff00 & v) >> 8);
                a[1] = (char) (0xff & v);

                a[2] = (char) ((0xff00 & vn) >> 8);
                a[3] = (char) (0xff & vn);

                a[4] = (char) ((0xff00 & w) >> 8);
                a[5] = (char) (0xff & w);

                a[6] = b;

                sC.serialSend("13A20040B09872", 5, a, 7, port->getFd());

            }
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	std::cout << "Ros shutdown, proceeding to close the controller." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

bool Controller::readSerial(){

    std_msgs::String msg;
    std::stringstream ss;
    memset(rx_buffer, 0, sizeof rx_buffer);

    bool error = false;
    fd_set readfs;
    struct timeval Timeout;

    FD_ZERO(&readfs);
    FD_SET(port->getFd(), &readfs);


    /* set timeout value within input loop */
    Timeout.tv_usec = 150000;  /* microseconds */
    Timeout.tv_sec  = 0;  /* seconds */

    int ret = select(port->getFd()+1, &readfs, NULL, NULL, &Timeout);

    if(ret) {

        int nr = read(port->getFd(), &rx_buffer , sizeof rx_buffer);

        if(nr > -1)  {

           if(nr!=RX_BUFFER_SIZE) {
               ss << "ERROR - Buffer Size" << std::endl;
               error = true;
           }
           else if(rx_buffer[0]!=0x7E){ //Testa Delimitador de Inicio de Mensagem
               ss << "ERROR - Starter Delimiter" << std::endl;
               error = true;
           }
           else if(rx_buffer[2]!=(RX_BUFFER_SIZE-4)) { //Testa Tamanho da msg maior que 14 (tamanho do cabeçalho)
               ss << "ERROR - Msg Size" << std::endl;
               error = true;
           }
           else if(rx_buffer[15]!=1){ //Testa Comando igual a 1
               ss << "ERROR - Command" << std::endl;
               error = true;
           }
           else if(((char) SerialCommunication::checkSum(&rx_buffer[3],(int) rx_buffer[2]))!=rx_buffer[RX_BUFFER_SIZE-1]){ //Verifica Checksum
               ss << "ERROR - Buffer Size" << std::endl;
               error = true;
           }
           else {
               //Tudo ocorreu bem na leitura
               return true;
           }

           if(error) {
               error = false;
               msg.data = ss.str();
               log(Error,msg.data);
           }

       }

       else {
           ss << "Error reading: " << nr << " " << errno << " " << strerror(errno) << std::endl;
           msg.data = ss.str();
           log(Error,msg.data);
       }
    }
    else
    {
        ss << "Error: Timeout" << std::endl;
        msg.data = ss.str();
        log(Error,msg.data);
    }

    return false;
}

void Controller::publisher(int dt)
{

	int16_t m_velocity = (((int) (unsigned char) rx_buffer[16])*256 + ((int) (unsigned char) rx_buffer[17]));
	pkg_data.m1_velocity = (0x01 & ((int) rx_buffer[36]))?m_velocity:(-1*m_velocity);
	m_velocity = (((int) (unsigned char) rx_buffer[18])*256 + ((int) (unsigned char) rx_buffer[19]));
	pkg_data.m2_velocity = (0x02 & ((int) rx_buffer[36]))?m_velocity:(-1*m_velocity);
	m_velocity = (((int) (unsigned char) rx_buffer[20])*256 + ((int) (unsigned char) rx_buffer[21]));
	pkg_data.m3_velocity = (0x04 & ((int) rx_buffer[36]))?m_velocity:(-1*m_velocity);
	pkg_data.m1_current = ((int) (unsigned char) rx_buffer[22])*256 + ((int) (unsigned char) rx_buffer[23]);
	pkg_data.m2_current = ((int) (unsigned char) rx_buffer[24])*256 + ((int) (unsigned char) rx_buffer[25]);
	pkg_data.m3_current = ((int) (unsigned char) rx_buffer[26])*256 + ((int) (unsigned char) rx_buffer[27]);
  pkg_data.x_acelleration = ((int) (unsigned char) rx_buffer[28])*256 + ((int) (unsigned char) rx_buffer[29]);
	pkg_data.y_acelleration = ((int) (unsigned char) rx_buffer[30])*256 + ((int) (unsigned char) rx_buffer[31]);
	pkg_data.angular_velocity = ((int) (unsigned char) rx_buffer[32])*256 + ((int) (unsigned char) rx_buffer[33]);
	pkg_data.compass = ((int) (unsigned char) rx_buffer[34])*256 + ((int) (unsigned char) rx_buffer[35]);
	pkg_data.m1_dutycycle = (int) (unsigned char) rx_buffer[37];
	pkg_data.m2_dutycycle = (int) (unsigned char) rx_buffer[38];
	pkg_data.m3_dutycycle = (int) (unsigned char) rx_buffer[39];
	int16_t m_setpoint = (((int) (unsigned char) rx_buffer[40])*256 + ((int) (unsigned char) rx_buffer[41]));
	pkg_data.m1_setpoint = (0x01 & ((int) rx_buffer[46]))?m_setpoint:(-1*m_setpoint);
	m_setpoint = (((int) (unsigned char) rx_buffer[42])*256 + ((int) (unsigned char) rx_buffer[43]));
	pkg_data.m2_setpoint = (0x02 & ((int) rx_buffer[46]))?m_setpoint:(-1*m_setpoint);
	m_setpoint = (((int) (unsigned char) rx_buffer[44])*256 + ((int) (unsigned char) rx_buffer[45]));
	pkg_data.m3_setpoint = (0x04 & ((int) rx_buffer[46]))?m_setpoint:(-1*m_setpoint);
	pkg_data.m1_inc_control_signal = dU[0];
	pkg_data.m2_inc_control_signal = dU[1];
	pkg_data.m3_inc_control_signal = dU[2];
	pkg_data.m1_control_signal = U[0];
	pkg_data.m2_control_signal = U[1];
	pkg_data.m3_control_signal = U[2];
	pkg_data.V = vel[0];
  pkg_data.Vn = vel[1];
  pkg_data.W = vel[2];
	pkg_data.delta_time = dt;
  pkg_data.X_robot = xr;
  pkg_data.Y_robot = yr;
  pkg_data.Theta_robot = theta;
  pkg_data.Vref = Vref[0];
  pkg_data.Vnref = Vref[1];
  pkg_data.Wref = Vref[2];

	Q_EMIT updateList();

}

void Controller::smith_predictor()
{

  Mat QSI;

	old_x[0] = x[0];
	old_x[1] = x[1];
	old_x[2] = x[2];

	//Encontrando o erro entre os valores lidos e os valores do modelo
	difvel[0] = vel[0] - velm[0];
	difvel[1] = vel[1] - velm[1];
	difvel[2] = vel[2] - velm[2];

	//Calculando Novas Velocidades do modelo
	new_velm[0] = Ad[0][0]*velm[0] + Ad[0][1]*velm[1] + Ad[0][2]*velm[2];
	new_velm[0] += Bd[0][0]*U[0] + Bd[0][1]*U[1] + Bd[0][2]*U[2];

	new_velm[1] = Ad[1][0]*velm[0] + Ad[1][1]*velm[1] + Ad[1][2]*velm[2];
	new_velm[1] += Bd[1][0]*U[0] + Bd[1][1]*U[1] + Bd[1][2]*U[2];

	new_velm[2] = Ad[2][0]*velm[0] + Ad[2][1]*velm[1] + Ad[2][2]*velm[2];
	new_velm[2] += Bd[2][0]*U[0] + Bd[2][1]*U[1] + Bd[2][2]*U[2];

	velm[0] = new_velm[0];
	velm[1] = new_velm[1];
	velm[2] = new_velm[2];

	//Filtro
  xFiltrado[0] = difvel[0]*(1-alpha) +alpha*xFiltrado[0];
  xFiltrado[1] = difvel[1]*(1-alpha) +alpha*xFiltrado[1];
  xFiltrado[2] = difvel[2]*(1-alpha) +alpha*xFiltrado[2];

	//Encontrando o vetor de estados após filtragem
	x[0] = velm[0] + xFiltrado[0];
	x[1] = velm[1] + xFiltrado[1];
	x[2] = velm[2] + xFiltrado[2];

  QSI = (cv::Mat_<float>(6,1) << 0, 0, 0, 0, 0, 0);

  QSI.at<float>(0,0)= x[0]-old_x[0];
  QSI.at<float>(1,0)= x[1]-old_x[1];
  QSI.at<float>(2,0)= x[2]-old_x[2];
  QSI.at<float>(3,0)= x[0];
  QSI.at<float>(4,0)= x[1];
  QSI.at<float>(5,0)= x[2];

  //Calculando a ação de controle preditivo sem restrições
  Mat K = ((BB.t()*CC.t()*Q.t()*CC*BB+R).inv())*(BB.t()*CC.t()*Q.t());

  K = K*(W-CC*AA*QSI);

  dU[0] = K.at<float>(0,0);
  dU[1] = K.at<float>(1,0);
  dU[2] = K.at<float>(2,0);

  QSI.release();
  K.release();

}

void Controller::optmum_predictor()
{

    Mat QSI;

    //Encontrando o vetor de estados x'(k+1|k)
    x[0] = Ag[0][0]*(vel[0]-old_vel[0]) + Ag[0][1]*(vel[1]- old_vel[1]) + Ag[0][2]*(vel[2]-old_vel[2]);
    x[0] += Ag[0][3]*(vel[0]) + Ag[0][4]*(vel[1]) + Ag[0][5]*(vel[2]);
    x[0] += Bg[0][0]*dU[0] + Bg[0][1]*dU[1] + Bg[0][2]*dU[2];

    x[1] = Ag[1][0]*(vel[0]-old_vel[0]) + Ag[1][1]*(vel[1]- old_vel[1]) + Ag[1][2]*(vel[2]-old_vel[2]);
    x[1] += Ag[1][3]*(vel[0]) + Ag[1][4]*(vel[1]) + Ag[1][5]*(vel[2]);
    x[1] += Bg[1][0]*dU[0] + Bg[1][1]*dU[1] + Bg[1][2]*dU[2];

    x[2] = Ag[2][0]*(vel[0]-old_vel[0]) + Ag[2][1]*(vel[1]- old_vel[1]) + Ag[2][2]*(vel[2]-old_vel[2]);
    x[2] += Ag[2][3]*(vel[0]) + Ag[2][4]*(vel[1]) + Ag[2][5]*(vel[2]);
    x[2] += Bg[2][0]*dU[0] + Bg[2][1]*dU[1] + Bg[2][2]*dU[2];

    x[3] = Ag[3][0]*(vel[0]-old_vel[0]) + Ag[3][1]*(vel[1]- old_vel[1]) + Ag[3][2]*(vel[2]-old_vel[2]);
    x[3] += Ag[3][3]*(vel[0]) + Ag[3][4]*(vel[1]) + Ag[3][5]*(vel[2]);
    x[3] += Bg[3][0]*dU[0] + Bg[3][1]*dU[1] + Bg[3][2]*dU[2];

    x[4] = Ag[4][0]*(vel[0]-old_vel[0]) + Ag[4][1]*(vel[1]- old_vel[1]) + Ag[4][2]*(vel[2]-old_vel[2]);
    x[4] += Ag[4][3]*(vel[0]) + Ag[4][4]*(vel[1]) + Ag[4][5]*(vel[2]);
    x[4] += Bg[4][0]*dU[0] + Bg[4][1]*dU[1] + Bg[4][2]*dU[2];

    x[5] = Ag[5][0]*(vel[0]-old_vel[0]) + Ag[5][1]*(vel[1]- old_vel[1]) + Ag[5][2]*(vel[2]-old_vel[2]);
    x[5] += Ag[5][3]*(vel[0]) + Ag[5][4]*(vel[1]) + Ag[5][5]*(vel[2]);
    x[5] += Bg[5][0]*dU[0] + Bg[5][1]*dU[1] + Bg[5][2]*dU[2];

    QSI = (cv::Mat_<float>(6,1) << 0, 0, 0, 0, 0, 0);

    QSI.at<float>(0,0)= x[0];
    QSI.at<float>(1,0)= x[1];
    QSI.at<float>(2,0)= x[2];
    QSI.at<float>(3,0)= x[3];
    QSI.at<float>(4,0)= x[4];
    QSI.at<float>(5,0)= x[5];

    Mat K = ((BB.t()*CC.t()*Q.t()*CC*BB+R).inv())*(BB.t()*CC.t()*Q.t());

    K = K*(W-CC*AA*QSI);

    dU[0] = K.at<float>(0,0);
    dU[1] = K.at<float>(1,0);
    dU[2] = K.at<float>(2,0);

    QSI.release();
    K.release();

}

void Controller::log( const LogLevel &level, const std::string &msg)
{
    logging_model.insertRows(logging_model.rowCount(),1);
    std::stringstream logging_model_msg;
    switch ( level )
    {
        case(Debug) :
        {
                ROS_DEBUG_STREAM(msg);
                logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
                break;
        }
        case(Info) :
        {
                ROS_INFO_STREAM(msg);
                logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
                break;
        }
        case(Warn) :
        {
                ROS_WARN_STREAM(msg);
                logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
                break;
        }
        case(Error) :
        {
                ROS_ERROR_STREAM(msg);
                logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
                break;
        }
        case(Fatal) :
        {
                ROS_FATAL_STREAM(msg);
                logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
                break;
        }
    }
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void Controller::trajetoria(int time) {

    float t = time/1000.0;

    xr += (vel[0]*cos(theta) - vel[1]*sin(theta))*t;
    yr += (vel[0]*sin(theta) + vel[1]*cos(theta))*t;
    theta += vel[2]*t;

    TrajectoryControl(xr, yr, theta);

    W.release();

    if(TrajOK==1) {

        W = Mat::zeros(3*HP, 1, CV_32F);

        Vref[0] = 0;
        Vref[1] = 0;
        Vref[2] = 0;

    }
    else {

        W.create(0,0,CV_32F);

        for(int i = 0; i < HP; i++){
                int z;

                if(Index+i<TrajPoints-1)
                    z = Index+i;
                else
                    z = TrajPoints-2;

                Mat r = referenciaFutura(z);
                W.push_back(r);
        }

        //Ainda utilizado devido ao log
        Vref[0] = W.at<float>(0,0);
        Vref[1] = W.at<float>(1,0);
        Vref[2] = W.at<float>(2,0);

    }

}

Mat Controller::referenciaFutura(int index){

    float e[3];
    float phi;
    float Vnav = 0.3;

    phi = atan2((TrajY[index+1] - yr),(TrajX[index+1] - xr));

    e[0] = Vnav*cos(phi);
    e[1] = Vnav*sin(phi);
    e[2] = TrajTheta[index+1] - theta;

    if (fabs(e[2]) < Rad(0.005))
        e[2] = 0;

    Mat r = (cv::Mat_<float>(3,1) << cos(theta)*e[0] + sin(theta)*e[1],
                                        -sin(theta)*e[0] + cos(theta)*e[1],
                                        e[2]);

    return r;
}

void Controller::TrajectoryControl(double x, double y, double theta)
{

  double ang;

  ang = fabs(DiffAngle(atan2((y-TrajY[Index+1]),(x-TrajX[Index+1])),atan2((TrajY[Index+1]-TrajY[Index]),(TrajX[Index+1]-TrajX[Index]))));

  if (ang < PI/2) {

  if (Index == TrajPoints-1) {
     TrajOK=1;
  }
  else {
     Index++;
  }

  if (Index == TrajPoints-2) {
    if (Dist((x-TrajX[TrajPoints-1]),(y-TrajY[TrajPoints-1])) < 0.05) {
      TrajOK=1;
    }
  }

 }

}

double Controller::DiffAngle(double a1, double a2)
{

    double result;

    result = a1-a2;
    if (result<0) result = -fmod(-result,2*PI);
    if (result<-PI) result = result+2*PI;
        else result = fmod(result,2*PI);
    if (result>PI) result = result-2*PI;

    return result;
}

double Controller::Dist(double x, double y)
{
  return sqrt(x*x+y*y);
}

double Controller::Rad(double xw){
  return xw*(PI/180);
}

void Controller::print(Mat mat, int prec) {

    std_msgs::String msg;
    std::stringstream ss;

    for(int i=0; i<mat.size().height; i++)
    {
        ss << "[";
        for(int j=0; j<mat.size().width; j++)
        {
            ss << std::setprecision(prec) << mat.at<float>(i,j);
            if(j != mat.size().width-1)
                ss << ", ";
            else
                ss << "]" << endl;
        }
    }

    ss << "\n\n";

    msg.data = ss.str();
    log(Info,msg.data);

}

}  // namespace ros_gui
