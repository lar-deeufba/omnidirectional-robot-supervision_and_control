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

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ros_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

//Para matrizes de ponderaзгo: Q 100*I e R = I
const float Controller::Kdlqr[3][6] = {
    { 0.0000,   -7.3404,    0.1982,    0.0000,   -5.0455,    0.3356},
    { 6.3570,    3.6702,    0.1982,    4.3695,    2.5227,    0.3356},
    {-6.3570,    3.6702,    0.1982,   -4.3695,    2.5227,    0.3356}};

/*Referentes ao preditor Otimo*/
const float Controller::Ag[6][6] = {
    {0.8218,         0,         0,         0,         0,         0},
    {     0,    0.8218,         0,         0,         0,         0},
    {     0,         0,    0.5888,         0,         0,         0},
    {0.8218,         0,         0,    1.0000,         0,         0},
    {     0,    0.8218,         0,         0,    1.0000,         0},
    {     0,         0,    0.5888,         0,         0,    1.0000}};

const float Controller::Bg[6][3] = {
    {      0,    0.0400,   -0.0400},
    {-0.0461,    0.0231,    0.0231},
    { 0.9868,    0.9868,    0.9868},
    {      0,    0.0400,   -0.0400},
    {-0.0461,    0.0231,    0.0231},
    { 0.9868,    0.9868,    0.9868}};

/*Referentes ao preditor de Smith*/
const float Controller::Ad[3][3] = {
    {0.8218,         0,         0},
    {     0,    0.8218,         0},
    {     0,         0,    0.5888}};

const float Controller::Bd[3][3] = {
    {        0,    0.0400,   -0.0400},
    {    -0.0461,    0.0231,    0.0231},
    {    0.9868,    0.9868,    0.9868}};

const float Controller::alpha = 0.85;

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
    TrajPoints = 23;

//Circulo:
   TrajX[0] = 0.0000;
   TrajY[0] = 0.0000;
   TrajTheta[0] = 0;//
   TrajX[1] = 0.0371;
   TrajY[1] = 0.2698;
   TrajTheta[1] = 0;//
   TrajX[2] = 0.1456;
   TrajY[2] = 0.5196;
   TrajTheta[2] = 0;//
   TrajX[3] = 0.3174;
   TrajY[3] = 0.7308;
   TrajTheta[3] = 0;//
   TrajX[4] = 0.5399;
   TrajY[4] = 0.8879;
   TrajTheta[4] = 0;//
   TrajX[5] = 0.7965;
   TrajY[5] = 0.9791;
   TrajTheta[5] = 0;//
   TrajX[6] = 1.0682;
   TrajY[6] = 0.9977;
   TrajTheta[6] = 0;//
   TrajX[7] = 1.3349;
   TrajY[7] = 0.9423;
   TrajTheta[7] = 0;//
   TrajX[8] = 1.5767;
   TrajY[8] = 0.8170;
   TrajTheta[8] = 0;//
   TrajX[9] = 1.7757;
   TrajY[9] = 0.6311;
   TrajTheta[9] = 0;//
   TrajX[10] = 1.9172;
   TrajY[10] = 0.3984;
   TrajTheta[10] = 0;//
   TrajX[11] = 1.9907;
   TrajY[11] =  0.1362;
   TrajTheta[11] = 0;//
   TrajX[12] = 1.9907;
   TrajY[12] = -0.1362;
   TrajTheta[12] = 0;//
   TrajX[13] = 1.9172;
   TrajY[13] = -0.3984;
   TrajTheta[13] = 0;//
   TrajX[14] = 1.7757;
   TrajY[14] = -0.6311;
   TrajTheta[14] = 0;//
   TrajX[15] = 1.5767;
   TrajY[15] = -0.8170;
   TrajTheta[15] = 0;//
   TrajX[16] = 1.3349;
   TrajY[16] = -0.9423;
   TrajTheta[16] = 0;//
   TrajX[17] = 1.0682;
   TrajY[17] = -0.9977;
   TrajTheta[17] = 0;//
   TrajX[18] = 0.7965;
   TrajY[18] = -0.9791;
   TrajTheta[18] = 0;//
   TrajX[19] = 0.5399;
   TrajY[19] = -0.8879;
   TrajTheta[19] = 0;//
   TrajX[20] = 0.3174;
   TrajY[20] = -0.7308;
   TrajTheta[20] = 0;//
   TrajX[21] = 0.1456;
   TrajY[21] = -0.5196;
   TrajTheta[21] = 0;//
   TrajX[22] = 0.0371;
   TrajY[22] =  -0.2698;
   TrajTheta[22] = 0;//
   TrajX[23] = 0.0000;
   TrajY[23] = 0.0000;
   TrajTheta[23] = 0;//

   /*------------------------------------------------*/

   xr = yr = theta = 0;

   Vref[0] = 0; //m/s
   Vref[1] = 0;
   Vref[2] = 0; //rad/s

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

            trajetoria(t2.restart());

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
            //                vel[0] = 0*vel[0] + 0.5774*vel[1] - 0.5774*vel[2];
            //                vel[1] = -0.6667*vel[0] + 0.3333*vel[1] + 0.3333*vel[2];
            //                vel[2] = 3.3333*vel[0] + 3.3333*vel[1] + 3.3333*vel[2];
            //Multiplica a matriz acima comentada por 0,0505
            //para obter a vel linear a partir de velrad
            vel[0] = 0*velrad[0] + 0.0292*velrad[1] - 0.0292*velrad[2];
            vel[1] = -0.0337*velrad[0] + 0.0168*velrad[1] + 0.0168*velrad[2];
            vel[2] = 0.1683*velrad[0] + 0.1683*velrad[1] + 0.1683*velrad[2];

            //Controller
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

            // write the new control action on the serial port
            sC.serialSend("13A20040B09872", 2, PWM, 4, port->getFd());
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
    //std::cout << "2" << std::endl;

    int ret = select(port->getFd()+1, &readfs, NULL, NULL, &Timeout);
    //std::cout << "asfsdf " << ret << " fsdfsdsfd" << std::endl;

    if(ret) {

        //std::cout << "3" << std::endl;

        int nr = read(port->getFd(), &rx_buffer , sizeof rx_buffer);

        //std::cout << "4" << std::endl;

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

               //Clear para limpa buffer
               //port->clearSerialData(0);
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

	//Calculando a ação de controle
	dU[0] = -Kdlqr[0][0]*(x[0]-old_x[0]) - Kdlqr[0][1]*(x[1]- old_x[1]) - Kdlqr[0][2]*(x[2]-old_x[2]);
	dU[0] += Kdlqr[0][3]*(Vref[0] - x[0]) + Kdlqr[0][4]*(Vref[1] - x[1]) + Kdlqr[0][5]*(Vref[2] - x[2]);

	dU[1] = -Kdlqr[1][0]*(x[0]-old_x[0]) - Kdlqr[1][1]*(x[1]- old_x[1]) - Kdlqr[1][2]*(x[2]-old_x[2]);
	dU[1] += Kdlqr[1][3]*(Vref[0] - x[0]) + Kdlqr[1][4]*(Vref[1] - x[1]) + Kdlqr[1][5]*(Vref[2] - x[2]);

	dU[2] = -Kdlqr[2][0]*(x[0]-old_x[0]) - Kdlqr[2][1]*(x[1]- old_x[1]) - Kdlqr[2][2]*(x[2]-old_x[2]);
	dU[2] += Kdlqr[2][3]*(Vref[0] - x[0]) + Kdlqr[2][4]*(Vref[1] - x[1]) + Kdlqr[2][5]*(Vref[2] - x[2]);
}

void Controller::optmum_predictor()
{
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

    //Calculando a ação de controle utilizando o LQR
    dU[0] = -Kdlqr[0][0]*x[0] - Kdlqr[0][1]*x[1] - Kdlqr[0][2]*x[2];
    dU[0] += Kdlqr[0][3]*(Vref[0] - x[3]) + Kdlqr[0][4]*(Vref[1] - x[4]) + Kdlqr[0][5]*(Vref[2] - x[5]);

    dU[1] = -Kdlqr[1][0]*x[0] - Kdlqr[1][1]*x[1] - Kdlqr[1][2]*x[2];
    dU[1] += Kdlqr[1][3]*(Vref[0] - x[3]) + Kdlqr[1][4]*(Vref[1] - x[4]) + Kdlqr[1][5]*(Vref[2] - x[5]);

    dU[2] = -Kdlqr[2][0]*x[0] - Kdlqr[2][1]*x[1] - Kdlqr[2][2]*x[2];
    dU[2] += Kdlqr[2][3]*(Vref[0] - x[3]) + Kdlqr[2][4]*(Vref[1] - x[4]) + Kdlqr[2][5]*(Vref[2] - x[5]);
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

    float e[3];
    float phi;
    float Vnav = 0.3;

    float t = time/1000.0;

    xr += (vel[0]*cos(theta) - vel[1]*sin(theta))*t;
    yr += (vel[0]*sin(theta) + vel[1]*cos(theta))*t;
    theta += vel[2]*t;

    TrajectoryControl(xr, yr, theta);

    if(TrajOK==1) {
        Vref[0] = 0;
        Vref[1] = 0;
        Vref[2] = 0;
    }
    else {

    phi = atan2((TrajY[Index+1] - yr),(TrajX[Index+1] - xr));

    e[0] = Vnav*cos(phi);
    e[1] = Vnav*sin(phi);
    e[2] = TrajTheta[Index+1] - theta;
    if (fabs(e[2]) < Rad(0.005))
        e[2] = 0;

    Vref[0] = cos(theta)*e[0] + sin(theta)*e[1];
    Vref[1] = -sin(theta)*e[0] + cos(theta)*e[1];
    Vref[2] = e[2];

    }

}

void Controller::TrajectoryControl(double x, double y, double theta)
{

double ang;

 ang = fabs(DiffAngle(atan2((y-TrajY[Index+1]),(x-TrajX[Index+1])),atan2((TrajY[Index+1]-TrajY[Index]),(TrajX[Index+1]-TrajX[Index]))));

 if (ang < PI/2){

   if (Index == TrajPoints-1){
       TrajOK=1;
   }else{
       Index++;
   }

    if (Index == TrajPoints-2){
      if (Dist((x-TrajX[TrajPoints-1]),(y-TrajY[TrajPoints-1])) < 0.001)
      TrajOK=1;
    }
 };


};

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

}  // namespace ros_gui
