#include "ArduUWBPlugin.hh"

#include "ArduRotorNormPlugin.hh"
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Filter.hh>
#include <mutex>
#include <sdf/sdf.hh>
#include <string>
#include <vector>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ArduUWBPlugin)

/// \brief Flight Dynamics Model packet that is sent back to the ArduPilot
struct uwbPacket {
    /// \brief packet timestamp
    double timestamp;

    /// \brief Model velocity in NED frame
    double velocityXYZ[3];

    /// \brief Model position in NED frame
    double positionXYZ[3];

    double beaconXYZ[4][3];

    double beaconDistance[4];
};

class gazebo::ArduPilotTcpSocketPrivate {

public:
    ArduPilotTcpSocketPrivate()
    {
        printf("ArduPilotTcpSocketPrivate");
        if ((fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
            printf("create socket error: %s(errno: %d)\n", strerror(errno), errno);
        }
    }

public:
    ~ArduPilotTcpSocketPrivate()
    {
        if (fd != -1) {
            ::close(fd);
            fd = -1;
        }
    }

public:
    bool Connect(const char* _address, const uint16_t _port)
    {
        struct sockaddr_in sockaddr;

        this->MakeSockAddr(_address, _port, sockaddr);

        if (connect(this->fd, (struct sockaddr*)&sockaddr, sizeof(sockaddr)) != 0) {
            // shutdown(this->fd, 0);

            // close(this->fd);

            return false;
        }

        int one = 1;
        setsockopt(this->fd, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&one), sizeof(one));

        fcntl(this->fd, F_SETFL, fcntl(this->fd, F_GETFL, 0) | O_NONBLOCK);

        return true;
    }

    void MakeSockAddr(const char* _address, const uint16_t _port, struct sockaddr_in& _sockaddr)
    {
        memset(&_sockaddr, 0, sizeof(_sockaddr));

        _sockaddr.sin_port = htons(_port);
        _sockaddr.sin_family = AF_INET;
        _sockaddr.sin_addr.s_addr = inet_addr(_address);
    }

public:
    ssize_t Send(const void* _buf, size_t _size) { return send(this->fd, _buf, _size, 0); }

    /// \brief Socket handle
private:
    int fd;
};

class gazebo::ArduUWBPluginPrivate {
    /// \brief Pointer to the update event connection.
public:
    event::ConnectionPtr updateConnection;

    /// \brief String of the model name;
public:
    std::string modelName;

    /// \brief Pointer to the model;
public:
    physics::ModelPtr model;

    /// \brief Ardupilot address
public:
    std::string ardu_addr;

    /// \brief Ardupilot port for receiver socket
public:
    uint16_t ardu_port;

public:
    ArduPilotTcpSocketPrivate tcp_client;

public:
    bool connect_state;

public:
    uint16_t connect_state_cnt;

public:
    uwbPacket pkg;

public:
    unsigned char send_length;
    unsigned char send_buf[47];
};

ArduUWBPlugin::ArduUWBPlugin()
    : dataPtr(new ArduUWBPluginPrivate)
{
}

/////////////////////////////////////////////////
ArduUWBPlugin::~ArduUWBPlugin() { }

void ArduUWBPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_model, "ArduUWBPlugin _model pointer is null");
    GZ_ASSERT(_sdf, "ArduUWBPlugin _sdf pointer is null");

    this->dataPtr->model = _model;
    this->dataPtr->modelName = this->dataPtr->model->GetName();

    // modelXYZToAirplaneXForwardZDown brings us from gazebo model frame:
    // x-forward, y-right, z-down
    // to the aerospace convention: x-forward, y-left, z-up
    this->modelXYZToAirplaneXForwardZDown = ignition::math::Pose3d(0, 0, 0, 0, 0, 0);
    if (_sdf->HasElement("modelXYZToAirplaneXForwardZDown")) {
        this->modelXYZToAirplaneXForwardZDown = _sdf->Get<ignition::math::Pose3d>("modelXYZToAirplaneXForwardZDown");
    }

    // gazeboXYZToNED: from gazebo model frame: x-forward, y-right, z-down
    // to the aerospace convention: x-forward, y-left, z-up
    this->gazeboXYZToNED = ignition::math::Pose3d(0, 0, 0, IGN_PI, 0, 0);
    if (_sdf->HasElement("gazeboXYZToNED")) {
        this->gazeboXYZToNED = _sdf->Get<ignition::math::Pose3d>("gazeboXYZToNED");
    }

    this->dataPtr->connect_state = false;
    this->dataPtr->connect_state_cnt = 0;

    this->dataPtr->updateConnection
        = event::Events::ConnectWorldUpdateBegin(std::bind(&ArduUWBPlugin::OnUpdate, this)); //
    //     std::bind在functional中

    // Initialise ardupilot sockets
    this->dataPtr->ardu_addr = _sdf->Get("ardu_addr", static_cast<std::string>("127.0.0.1")).first;
    this->dataPtr->ardu_port = _sdf->Get("ardu_port", static_cast<uint32_t>(5762)).first;
}

/////////////////////////////////////////////////
bool ArduUWBPlugin::InitArduUWBSockets() const
{
    if (!this->dataPtr->tcp_client.Connect(this->dataPtr->ardu_addr.c_str(), this->dataPtr->ardu_port)) {
        gzerr << "[" << this->dataPtr->modelName << "] "
              << "failed to bind with " << this->dataPtr->ardu_addr << ":" << this->dataPtr->ardu_port
              << " aborting plugin.\n";
        return false;
    }

    return true;
}

/////////////////////////////////////////////////
void ArduUWBPlugin::OnUpdate()
{
    if ((this->dataPtr->connect_state == false)) {
        if ((this->dataPtr->connect_state_cnt % 200 == 0)) {
            if (InitArduUWBSockets())
                this->dataPtr->connect_state = true;
        }
        this->dataPtr->connect_state_cnt++;

        return;
    }
    /////////////////////////////////////////////////
    this->dataPtr->connect_state_cnt++;

    if ((this->dataPtr->connect_state_cnt % 10 != 0)) {
        return;
    }

    /////////////////////////////////////////////////
    SetState();

    int16_t Dist[16];
    int16_t Rtls[3];

    memset(Dist, 0, sizeof(uint16_t) * 16);

    Rtls[0] = int16_t(this->dataPtr->pkg.positionXYZ[0] * 100);
    Rtls[1] = int16_t(this->dataPtr->pkg.positionXYZ[1] * 100);
    Rtls[2] = int16_t(this->dataPtr->pkg.positionXYZ[2] * 100);

    for (int i = 0; i < 4; i++) {
        Dist[i] = this->dataPtr->pkg.beaconDistance[i] * 100;
    }

    setProtocal(Dist, Rtls);

    this->dataPtr->tcp_client.Send(this->dataPtr->send_buf, this->dataPtr->send_length);
}

void ArduUWBPlugin::SetState() const
{

    const float BEACON_SPACING_NORTH = 10.0;
    const float BEACON_SPACING_EAST = 20.0;
    const float BEACON_SPACING_UP = 10.0;

    this->dataPtr->pkg.timestamp = this->dataPtr->model->GetWorld()->SimTime().Double();

    // asssumed that the imu orientation is:
    //   x forward
    //   y right
    //   z down

    // get inertial pose and velocity
    // position of the uav in world frame
    // this position is used to calcualte bearing and distance
    // from starting location, then use that to update gps position.
    // The algorithm looks something like below (from ardupilot helper
    // libraries):
    //   bearing = to_degrees(atan2(position.y, position.x));
    //   distance = math.sqrt(self.position.x**2 + self.position.y**2)
    //   (self.latitude, self.longitude) = util.gps_newpos(
    //    self.home_latitude, self.home_longitude, bearing, distance)
    // where xyz is in the NED directions.
    // Gazebo world xyz is assumed to be N, -E, -D, so flip some stuff
    // around.
    // orientation of the uav in world NED frame -
    // assuming the world NED frame has xyz mapped to NED,
    // imuLink is NED - z down

    // model world pose brings us to model,
    // which for example zephyr has -y-forward, x-left, z-up
    // adding modelXYZToAirplaneXForwardZDown rotates
    //   from: model XYZ
    //   to: airplane x-forward, y-left, z-down
    const ignition::math::Pose3d gazeboXYZToModelXForwardZDown
        = this->modelXYZToAirplaneXForwardZDown + this->dataPtr->model->WorldPose();

    // get transform from world NED to Model frame
    const ignition::math::Pose3d NEDToModelXForwardZUp = gazeboXYZToModelXForwardZDown - this->gazeboXYZToNED;

    // ROS_INFO_STREAM_THROTTLE(1, "ned to model [" << NEDToModelXForwardZUp << "]\n");

    // N
    this->dataPtr->pkg.positionXYZ[0] = NEDToModelXForwardZUp.Pos().X();

    // E
    this->dataPtr->pkg.positionXYZ[1] = NEDToModelXForwardZUp.Pos().Y();

    // D
    this->dataPtr->pkg.positionXYZ[2] = NEDToModelXForwardZUp.Pos().Z();

    // Get NED velocity in body frame *
    // or...
    // Get model velocity in NED frame
    const ignition::math::Vector3d velGazeboWorldFrame = this->dataPtr->model->GetLink()->WorldLinearVel();
    const ignition::math::Vector3d velNEDFrame = this->gazeboXYZToNED.Rot().RotateVectorReverse(velGazeboWorldFrame);

    this->dataPtr->pkg.velocityXYZ[0] = velNEDFrame.X();
    this->dataPtr->pkg.velocityXYZ[1] = velNEDFrame.Y();
    this->dataPtr->pkg.velocityXYZ[2] = velNEDFrame.Z();

    ////////////////////////////////////////

    ignition::math::Vector3d beaconXYZ[4];
    ignition::math::Vector3d beaconDistance[4];
    ignition::math::Vector3d vechicleXYZ;

    vechicleXYZ[0] = this->dataPtr->pkg.positionXYZ[0];
    vechicleXYZ[1] = this->dataPtr->pkg.positionXYZ[1];
    vechicleXYZ[2] = this->dataPtr->pkg.positionXYZ[2];

    beaconXYZ[0][0] = this->dataPtr->pkg.beaconXYZ[0][0] = -BEACON_SPACING_NORTH / 2;
    beaconXYZ[1][0] = this->dataPtr->pkg.beaconXYZ[1][0] = -BEACON_SPACING_NORTH / 2;
    beaconXYZ[2][0] = this->dataPtr->pkg.beaconXYZ[2][0] = BEACON_SPACING_NORTH / 2;
    beaconXYZ[3][0] = this->dataPtr->pkg.beaconXYZ[3][0] = BEACON_SPACING_NORTH / 2;

    beaconXYZ[0][1] = this->dataPtr->pkg.beaconXYZ[0][1] = -BEACON_SPACING_EAST / 2;
    beaconXYZ[1][1] = this->dataPtr->pkg.beaconXYZ[1][1] = BEACON_SPACING_EAST / 2;
    beaconXYZ[2][1] = this->dataPtr->pkg.beaconXYZ[2][1] = BEACON_SPACING_EAST / 2;
    beaconXYZ[3][1] = this->dataPtr->pkg.beaconXYZ[3][1] = -BEACON_SPACING_EAST / 2;

    beaconXYZ[0][2] = this->dataPtr->pkg.beaconXYZ[0][2] = -BEACON_SPACING_UP / 2;
    beaconXYZ[1][2] = this->dataPtr->pkg.beaconXYZ[1][2] = BEACON_SPACING_UP / 2;
    beaconXYZ[2][2] = this->dataPtr->pkg.beaconXYZ[2][2] = -BEACON_SPACING_UP / 2;
    beaconXYZ[3][2] = this->dataPtr->pkg.beaconXYZ[3][2] = BEACON_SPACING_UP / 2;

    beaconDistance[0] = vechicleXYZ - beaconXYZ[0];
    beaconDistance[1] = vechicleXYZ - beaconXYZ[1];
    beaconDistance[2] = vechicleXYZ - beaconXYZ[2];
    beaconDistance[3] = vechicleXYZ - beaconXYZ[3];

    this->dataPtr->pkg.beaconDistance[0] = beaconDistance[0].Length();
    this->dataPtr->pkg.beaconDistance[1] = beaconDistance[1].Length();
    this->dataPtr->pkg.beaconDistance[2] = beaconDistance[2].Length();
    this->dataPtr->pkg.beaconDistance[3] = beaconDistance[3].Length();
}

const unsigned char auchCRCHi[] = /* CRC锟斤拷位锟街节憋拷*/
    { 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
        0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00,
        0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
        0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00,
        0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80,
        0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00,
        0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
        0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00,
        0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
        0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00,
        0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
        0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00,
        0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
        0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00,
        0xC1, 0x81, 0x40 };

const unsigned char auchCRCLo[] = /* CRC锟斤拷位锟街节憋拷*/
    { 0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D,
        0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B,
        0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16,
        0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36,
        0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B,
        0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D,
        0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20,
        0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C,
        0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9,
        0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5, 0x77,
        0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52,
        0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A,
        0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F,
        0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41,
        0x81, 0x80, 0x40 };

unsigned int CRC_Calculate(unsigned char* pdata, unsigned char num)
{
    unsigned char uchCRCHi = 0xFF;
    unsigned char uchCRCLo = 0xFF;
    unsigned char uIndex;
    while (num--) {
        uIndex = uchCRCHi ^ *pdata++;
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];
    }
    return (uchCRCHi << 8 | uchCRCLo);
}

void ArduUWBPlugin::setProtocal(int16_t* Dist, int16_t* Rtls)
{
    unsigned int crc;
    unsigned int i;
    this->dataPtr->send_length = 0;
    this->dataPtr->send_buf[this->dataPtr->send_length++] = 0x01;
    this->dataPtr->send_buf[this->dataPtr->send_length++] = 0x03;
    this->dataPtr->send_buf[this->dataPtr->send_length++] = 0; //ÏÈ²»Ð´³¤¶È µÈ×îºóÔÙÐ´Èë

    this->dataPtr->send_buf[this->dataPtr->send_length++] = 0xAC;
    this->dataPtr->send_buf[this->dataPtr->send_length++] = 0xDA; //´ú±í±êÇ©Êý¾Ý°ü

    this->dataPtr->send_buf[this->dataPtr->send_length++] = 0x00;
    this->dataPtr->send_buf[this->dataPtr->send_length++] = 0x03;

    for (i = 0; i < 16; i++) {
        this->dataPtr->send_buf[this->dataPtr->send_length++] = Dist[i] >> 8;
        this->dataPtr->send_buf[this->dataPtr->send_length++] = Dist[i] & 0x00FF;
    }
    for (i = 0; i < 3; i++) {
        this->dataPtr->send_buf[this->dataPtr->send_length++] = Rtls[i] >> 8;
        this->dataPtr->send_buf[this->dataPtr->send_length++] = Rtls[i] & 0x00FF;
    }

    this->dataPtr->send_buf[2] = this->dataPtr->send_length - 3;

    crc = CRC_Calculate(this->dataPtr->send_buf, this->dataPtr->send_length);
    this->dataPtr->send_buf[this->dataPtr->send_length++] = crc / 256;
    this->dataPtr->send_buf[this->dataPtr->send_length++] = crc % 256;

    for (int i = 0; i < 47; i++) {
        printf("%x ", this->dataPtr->send_buf[i]);
    }
    printf("\r\n");

    // printf("crc=0x%x,0x%x\r\n", this->dataPtr->send_buf[45], this->dataPtr->send_buf[46]);
}