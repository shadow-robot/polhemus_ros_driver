/*
* Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
*/

#ifndef POLHEMUS_ROS_DRIVER_VIPER_PROTOCOL_H
#define POLHEMUS_ROS_DRIVER_VIPER_PROTOCOL_H

#include <algorithm>

#define VIPER_CMD_PREAMBLE 0x43525056
#define VIPER_PNO_PREAMBLE 0x50525056

#define SENSORS_PER_SEU   16
#define SOURCES_PER_SEU   4

typedef struct vpFrameInfo
{
    uint8_t * pF;
    uint32_t uiSize;
    uint32_t uiFCountRx;
    int32_t iFrameErr;
    uint64_t ts;
}
vpFrameInfo;

typedef struct __attribute__((packed)) viper_header_t
{
    uint32_t seuid;
    uint32_t cmd;
    uint32_t action;
    uint32_t arg1;
    uint32_t arg2;
}
viper_header_t;

typedef struct __attribute__((packed)) viper_frame_header_t
{
    uint32_t preamble;
    uint32_t size;
}
viper_frame_header_t;

typedef struct __attribute__((packed)) viper_full_header_t
{
    uint32_t preamble;
    uint32_t size;
    viper_header_t command_header;
}
viper_full_header_t;

typedef struct __attribute__((packed)) viper_pno_data_t
{
    float pos[3];
    float ori[4];
}
viper_pno_data_t;

typedef struct __attribute__((packed)) viper_pno_data_a_t
{
    float pos[3];
    int16_t ori[4];
    int16_t acc;
}
viper_pno_data_a_t;

typedef struct __attribute__((packed)) viper_sensor_frame_info_t
{
    uint32_t    bfSnum       : 7;
    uint32_t    bfSvirt      : 1;
    uint32_t    bfPosUnits   : 2;
    uint32_t    bfOriUnits   : 2;
    uint32_t    bfBtnState0  : 1;
    uint32_t    bfBtnState1  : 1;
    uint32_t    bfDistortion : 8;
    uint32_t    bfAuxInput   : 10;
}
viper_sensor_frame_info_t;

typedef struct __attribute__((packed)) viper_sensor_frame_data_t
{
    viper_sensor_frame_info_t SFinfo;  // 4 bytes
    viper_pno_data_t pno;    // 28 bytes
}
viper_sensor_frame_data_t;

typedef struct __attribute__((packed)) viper_sensor_frame_data_a_t
{
    viper_sensor_frame_info_t SFinfo;  // 4 bytes
    viper_pno_data_a_t pno_a;    // 28 bytes
}
viper_sensor_frame_data_a_t;

typedef struct __attribute__((packed)) viper_hpo_info_t
{
    uint32_t bfPnoMode : 4;
    uint32_t bfReserved0 : 4;
    uint32_t bfReserved1 : 8;
    uint32_t bfReserved2 : 16;
}
viper_hpo_info_t;

typedef struct __attribute__((packed)) viper_pno_header_t
{
    uint32_t seuid;
    uint32_t frame;
    viper_hpo_info_t hp_info;
    uint32_t sensor_count;
}
viper_pno_header_t;

typedef struct __attribute__((packed)) viper_pno_t
{
    viper_frame_header_t hdr;
    viper_pno_header_t seupno;
    viper_sensor_frame_data_t sarr[SENSORS_PER_SEU];
}
viper_pno_t;

typedef struct __attribute__((packed)) viper_frame_t
{
  viper_full_header_t full_header;
  uint8_t data[];
}
viper_frame_t;

typedef struct __attribute__((packed)) viper_pno_full_header_t
{
  uint32_t preamble;
  uint32_t size;
  viper_pno_header_t command_header;
}
viper_pno_full_header_t;

typedef struct __attribute__((packed)) viper_pno_frame_t
{
  uint32_t preamble;
  uint32_t size;
  viper_pno_header_t header;
  viper_pno_data_t pno_data[];
}
viper_pno_frame_t;

typedef enum viper_cmd_actions_e
{
  CMD_ACTION_SET = 0,
  CMD_ACTION_GET,
  CMD_ACTION_RESET,
  CMD_ACTION_ACK,
  CMD_ACTION_NAK,
  CMD_ACTION_MAX
}
viper_cmd_actions_e;

typedef enum viper_cmds_e
{
  CMD_HEMISPHERE,
  CMD_FILTER,
  CMD_TIP_OFFSET,
  CMD_INCREMENT,
  CMD_BORESIGHT,
  CMD_SENSOR_WHOAMI,
  CMD_FRAMERATE,
  CMD_UNITS,
  CMD_SRC_ROTATION,
  CMD_SYNC_MODE,
  CMD_STATION_MAP,
  CMD_STYLUS,
  CMD_SEUID,
  CMD_DUAL_OUTPUT,
  CMD_SERIAL_CONFIG,
  CMD_BLOCK_CFG,
  CMD_FRAME_COUNT,
  CMD_BIT,
  CMD_SINGLE_PNO,
  CMD_CONTINUOUS_PNO,
  CMD_WHOAMI,
  CMD_INITIALIZE,
  CMD_PERSIST,
  CMD_ENABLE_MAP,
  CMD_FTT_MODE,
  CMD_MAP_STATUS,
  CMD_SENSOR_BLOCKCFG,
  CMD_SOURCE_CFG,
  CMD_PREDFILTER_CFG,
  CMD_PREDFILTER_EXT,
  CMD_SRC_SELECT,
  CMD_SNS_ORIGIN,
  CMD_SNS_VIRTUAL,
  CMD_MAX
}
viper_cmds_e;

typedef struct __attribute__((packed)) viper_station_map_t
{
  union
  {
    uint32_t stamap;
    struct
    {
      uint32_t sensor_map : 16;
      uint32_t reserved1 : 8;
      uint32_t source_map : 4;
      uint32_t reserved2 : 4;
    }
    bf;
  };
}
viper_station_map_t;

typedef struct __attribute__((packed)) viper_units_config_t
{
  uint32_t pos_units;
  uint32_t ori_units;
}
viper_units_config_t;

typedef struct __attribute__((packed)) viper_hemisphere_config_t
{
    uint32_t track_enabled;
    float params[3];
}
viper_hemisphere_config_t;

typedef struct __attribute__((packed)) viper_src_select_cfg_t
{
    uint32_t src_select_map;
}
viper_src_select_cfg_t;

typedef struct __attribute__((packed)) viper_boresight_config_t
{
  float params[4];
}
viper_boresight_config_t;

typedef enum viper_ori_units_e
{
  ORI_EULER_DEGREE = 0,
  ORI_EULER_RADIAN,
  ORI_QUATERNION,
  ORI_MAX
}
viper_ori_units_e;

typedef enum viper_pos_units_e
{
  POS_INCH = 0,
  POS_FOOT,
  POS_CM,
  POS_METER,
  POS_MAX
}
viper_pos_units_e;

class CVPcmd: public viper_full_header_t
{
public:
  explicit CVPcmd(uint32_t pre = VIPER_CMD_PREAMBLE) :
      ppay(0), szpay(0)
  {
    Init(pre);
  }

  CVPcmd(const CVPcmd & rv)
  {
    *this = rv;
  }

  void * operator ()(CVPcmd & rv)
  {
    return reinterpret_cast<void*>(&rv.preamble);
  }

  const void * operator ()(const CVPcmd & rv)
  {
    return (const void*) &rv.preamble;
  }

  CVPcmd & operator =(const CVPcmd & rv)
  {
    memcpy(this, (const void *) &rv.preamble, sizeof(viper_full_header_t));
    return *this;
  }

  operator viper_full_header_t *()
  {
    return reinterpret_cast<viper_full_header_t *>(this);
  }

  void Init(uint32_t pre = VIPER_CMD_PREAMBLE)
  {
    ppay = 0;
    szpay = 0;
    preamble = pre;  // VIPER_CMD_PREAMBLE;

    size = sizeof(viper_header_t) + CRC_BYTES;
  }

  void Fill(uint32_t cmd, uint32_t act, uint32_t a1 = 0, uint32_t a2 = 0, void *pp = 0, uint32_t szp = 0)
  {
    command_header.seuid = SEUID;
    command_header.cmd = cmd;
    command_header.action = act;
    command_header.arg1 = a1;
    command_header.arg2 = a2;
    if (act == CMD_ACTION_SET)
    {
      ppay = pp;
      szpay = szp;
      size += szp;
    }
    else
    {
      ppay = 0;
      szpay = 0;
    }
  }

  void Prepare(uint8_t buf[], int & txbytes)
  {
    CVPcmd *ptx = reinterpret_cast<CVPcmd*>(buf);

    uint32_t crc_count = sizeof(viper_full_header_t);
    memcpy(ptx, (const void *) this, sizeof(viper_full_header_t));
    if (ppay)
    {
      memcpy(&buf[sizeof(viper_full_header_t)], ppay, szpay);
      crc_count += szpay;
    }

    uint32_t *pcrc = reinterpret_cast<uint32_t*>(&buf[crc_count]);
    *pcrc = CalcCRC_Bytes(buf, crc_count);

    txbytes = crc_count + sizeof(uint32_t);
  }

  static void crc16(uint32_t * crc, uint32_t data)
  {
    static const char op[16] = { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };
    data = (data ^ (*crc)) & 0xff;
    *crc >>= 8;

    if (op[data & 0xf] ^ op[data >> 4])
      *crc ^= 0xc001;

    data <<= 6;
    *crc ^= data;
    data <<= 1;
    *crc ^= data;

    return;
  }

  static uint32_t CalcCRC_Bytes(uint8_t *data, uint32_t count)
  {
    uint32_t crc;
    uint32_t n;

    crc = 0;
    for (n = 0; n < count; n++)
      crc16(&crc, data[n]);

    return crc;
  }

  void * ppay;
  uint32_t szpay;
};

class CStationMap: public viper_station_map_t
{
public:
  uint32_t sns_detected_count;  //<! Detected sensor count
  uint32_t src_detected_count;  //<! Detected source count
  uint32_t en_count;        //<! Enabled sensor count (enabled & detected)
  uint32_t en_map;          //<! Enabled sensor map.  For host use and maintenance only!

  CStationMap()
  {
    Init();
  }

  CStationMap(const CStationMap & rv)
  {
    stamap = rv.stamap;
    en_count = rv.en_count;
    en_map = rv.en_map;
    CountDetected();
  }

  explicit CStationMap(const viper_station_map_t * prv)
  {
    stamap = prv->stamap;
    CountDetected();
    en_count = sns_detected_count;
    en_map = bf.sensor_map;
  }

  operator viper_station_map_t *()
  {
    return reinterpret_cast<viper_station_map_t *>(this);
  }

  operator void *()
  {
    return reinterpret_cast<void *>(this);
  }

  bool operator ==(const viper_station_map_t *prv)
  {
    return (prv->stamap == stamap);
  }

  void Init()
  {
    memset(&stamap, 0, sizeof(viper_station_map_t));
    CountDetected();
    InitEnabled();
  }

  void CountDetected()
  {
    sns_detected_count = 0;
    for (int i = 0; i < SENSORS_PER_SEU; i++)
    {
      if ((1 << i) & bf.sensor_map)
        sns_detected_count++;
    }
    src_detected_count = 0;
    for (int i = 0; i < SOURCES_PER_SEU; i++)
    {
      if ((1 << i) & bf.source_map)
        src_detected_count++;
    }
  }

  void CountEnabled()
  {
    CountDetected();
    en_count = 0;
    for (int i = 0; i < SENSORS_PER_SEU; i++)
    {
      if ((1 << i) & (en_map))
        en_count++;
    }
  }

  void InitEnabled()
  {
    en_map = bf.sensor_map;
    CountEnabled();
  }

  void InitDefault()
  {
    memcpy(this, &Default, sizeof(viper_station_map_t));
    CountDetected();
    InitEnabled();
  }

  uint32_t SensorMap()
  {
    return bf.sensor_map;
  }

  uint32_t SourceMap()
  {
    return bf.source_map;
  }

  uint32_t & EnabledMap()
  {
    return en_map;
  }

  bool IsDetected(int32_t sns)
  {
    return ((1 << sns) & bf.sensor_map) != 0;
  }

  bool IsEnabled(int32_t sns)
  {
    return ((1 << sns) & (en_map & bf.sensor_map)) != 0;
  }

  bool IsSrcDetected(int32_t src)
  {
    return ((1 << src) & bf.source_map) != 0;
  }

  uint32_t SnsDetectedCount()
  {
    CountDetected();
    return sns_detected_count;
  }

  void SetEnabled(uint32_t s)
  {
    en_map &= (1 << s);
    CountEnabled();
  }

  static viper_station_map_t Default;
};

class CFrameInfo : public vpFrameInfo
{
public:
  CFrameInfo() { Init(); }

  CFrameInfo(uint8_t* p, uint32_t size, uint32_t FC, uint32_t FE)
  {
    Init();
    pF = p; uiSize = size;
    uiFCountRx = FC;
    iFrameErr = FE;
  }

  CFrameInfo(uint8_t* p, uint32_t size, uint32_t fc = 0, int32_t FE = 0, uint64_t ats = 0)
  {
    Init();
    pF = p; uiSize = size; uiFCountRx = fc; iFrameErr = FE; ts = ats;
  }

  explicit CFrameInfo(vpFrameInfo & rv)
  {
    memcpy(&pF, &rv.pF, sizeof(vpFrameInfo));
  }

  CFrameInfo & operator=(const CFrameInfo & rv)
  {
    pF = rv.pF;
    uiSize = rv.uiSize;
    uiFCountRx = rv.uiFCountRx;
    iFrameErr = rv.iFrameErr;
    ts = rv.ts;

    return *this;
  }

  void DeepCopy(uint8_t *pbuf, uint32_t uiBufsize, const CFrameInfo & rv)
  {
    uiSize = std::min<uint32_t>(uiBufsize, rv.uiSize);
    uiFCountRx = rv.uiFCountRx;
    iFrameErr = rv.iFrameErr;
    ts = rv.ts;

    if (pbuf)
    {
      pF = pbuf;
      memcpy(pF, rv.pF, uiSize);
    }
  }

  void Init()
  {
    pF = 0; uiSize = 0;
    uiFCountRx = 0;
    iFrameErr = 0;
    ts = 0;
  }

  uint32_t cmd() const
  {
    if (IsCmd())
      return (reinterpret_cast<viper_full_header_t*>(pF))->command_header.cmd;
    else
      return (uint32_t)-1;
  }

  uint32_t action() const
  {
    if (IsCmd())
      return (reinterpret_cast<viper_full_header_t*>(pF))->command_header.action;
    else
      return (uint32_t)-1;
  }

  uint32_t devid() const
  {
    if (IsCmd() || IsPno())
      return (reinterpret_cast<viper_full_header_t*>(pF))->command_header.seuid;
    else
      return (uint32_t)-1;
  }

  uint32_t devfc() const
  {
    if (IsNull() || IsCmd())
      return 0;
    else
      return (reinterpret_cast<viper_pno_full_header_t*>(pF))->command_header.frame;
  }

  int32_t err() const
  {
    return iFrameErr;
  }

  uint64_t & TS()
  {
    return ts;
  }

  bool IsCmd() const
  {
    if (IsNull())
      return false;
    else if ((reinterpret_cast<viper_frame_header_t*>(pF))->preamble == VIPER_CMD_PREAMBLE)
      return true;
    else
      return false;
  }

  bool IsPno() const
  {
    if (IsNull())
      return false;
    else if ((reinterpret_cast<viper_frame_header_t*>(pF))->preamble == VIPER_PNO_PREAMBLE)
      return true;
    else
      return false;
  }
  uint32_t Preamble() const
  {
    return (reinterpret_cast<viper_frame_header_t*>(pF))->preamble;
  }

  bool IsNull() const
  {
    return (pF == 0);
  }

  bool IsNak() const
  {
    if (IsCmd())
      return (action() == CMD_ACTION_NAK);

    else
      return false;
  }

  bool IsAck() const
  {
    if (IsCmd())
      return (action() == CMD_ACTION_ACK);

    else
      return false;
  }

  uint8_t * PCmdPayload()
  {
    if (IsCmd() && !IsNull())
      return &pF[sizeof(viper_full_header_t)];
    else
      return 0;
  }

  uint32_t CmdPayloadSize()
  {
    if (IsCmd() && !IsNull())
    {
      return (reinterpret_cast<viper_full_header_t*>(pF))->size - sizeof(viper_header_t) - CRC_BYTES;
    }
    else
      return 0;
  }

  uint8_t *PPnoBody()
  {
    if (IsPno() && !IsNull())
      return &pF[sizeof(viper_frame_header_t)];
    else
      return 0;
  }
  viper_sensor_frame_data_t *pSen(int32_t s = 0)
  {
    if (IsPno() && !IsNull())
      return reinterpret_cast<viper_sensor_frame_data_t *>
        (&pF[sizeof(viper_pno_full_header_t) + (s * sizeof(viper_sensor_frame_data_t))]);
    else
      return 0;
  }
  uint32_t PnoBodySize()
  {
    if (IsPno() && !IsNull())
    {
      return (reinterpret_cast<viper_pno_full_header_t*>(pF))->size - CRC_BYTES;
    }
    else
      return 0;
  }
};

class CVPSeuPno : public viper_pno_t
{
public:
  CVPSeuPno()
  {
    Init();
  }

  CVPSeuPno(const CVPSeuPno & rv)
  {
    memcpy(&hdr, &rv.hdr, sizeof(viper_pno_t));
  }

  explicit CVPSeuPno(const viper_pno_t * prv)
  {
    Init();
    memcpy(&hdr, prv, sizeof(viper_pno_t));
  }

  explicit CVPSeuPno(uint8_t *p)
  {
    Init();
    memcpy(&hdr, p, sizeof(viper_pno_t));
  }

  operator viper_pno_t * ()
  {
    return reinterpret_cast<viper_pno_t *>(this);
  }

  CVPSeuPno & operator= (const CVPSeuPno & rv)
  {
    memcpy(&hdr, &rv.hdr, sizeof(viper_pno_t));
    return *this;
  }

  CVPSeuPno & operator= (const viper_pno_t * prv)
  {
    memcpy(&hdr, prv, sizeof(viper_pno_t));
    return *this;
  }

  CVPSeuPno & operator= (uint8_t *p)
  {
    memcpy(&hdr, p, sizeof(viper_pno_t));
    return *this;
  }

  uint32_t Extractraw(uint8_t *p)
  {
    if (!p)
    return 0;

    viper_frame_header_t *ph = reinterpret_cast<viper_frame_header_t*>(p);
    if (ph->preamble != VIPER_PNO_PREAMBLE)
    return 0;

    Init();
    uint32_t index = 0;
    memcpy(&hdr, ph, sizeof(viper_frame_header_t)); index += sizeof(viper_frame_header_t);
    memcpy(&seupno, &p[index], sizeof(viper_pno_header_t)); index += sizeof(viper_pno_header_t);
    for (uint32_t i=0; i < seupno.sensor_count; i++)
    {
      memcpy(&sarr[i], &p[index], sizeof(viper_sensor_frame_data_t)); index += sizeof(viper_sensor_frame_data_t);
    }

    index += sizeof(CRC_BYTES);
    return index;
  }

  uint32_t Extractseupno(uint8_t *p)
  {
    if (!p)
    return 0;

    Init();
    uint32_t index = 0;

    memcpy(&seupno, &p[index], sizeof(viper_pno_header_t)); index += sizeof(viper_pno_header_t);
    for (uint32_t i = 0; i < seupno.sensor_count; i++)
    {
      memcpy(&sarr[i], &p[index], sizeof(viper_sensor_frame_data_t)); index += sizeof(viper_sensor_frame_data_t);
    }

    index += sizeof(CRC_BYTES);
    return index;
  }

  uint32_t SensorCount() const
  { return seupno.sensor_count;}

  uint32_t SensorMap() const
  {
    uint32_t map = 0;

    for (uint32_t i = 0; i < seupno.sensor_count; i++)
    {
      const viper_sensor_frame_data_t* pSD = &(sarr[i]);

      map |= (1 << pSD->SFinfo.bfSnum);
    }
    return map;
  }

  viper_sensor_frame_data_t * SensFrame(int i)
  {
    if (i < static_cast<int>(SensorCount()))
    return &(sarr[i]);
    else
    return 0;
  }

  viper_sensor_frame_data_a_t * SensFrameA(int i)
  {
    if (i < static_cast<int>(SensorCount()))
    return reinterpret_cast<viper_sensor_frame_data_a_t*>(&(sarr[i]));
    else
    return 0;
  }

  uint32_t Mode()
  {
    return seupno.hp_info.bfPnoMode;
  }

  void Init()
  {
    memset(&hdr, 0, sizeof(viper_pno_t));
  }
};

#endif  // POLHEMUS_ROS_DRIVER_VIPER_PROTOCOL_H
