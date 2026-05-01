# ComLink_Receiver Project

简介: 本工程用于实现任意EPS32开发板作为ComLink手柄接收机,用以控制其他边缘设备

---

**目前实现:**

- 接收ComLink广播数据
- ComLink自动配对 P2P通讯

**2026.5.1:**

- 增加了CRSF协议支持
  - GPIO17 - RX
  - GPIO18 - TX

**ToDo:**

- [ ] 私有协议
- [ ] 系列芯片兼容(32,S3,C3,C5,C6)
- [ ] 多接收机互联网络
- [ ] PWM协议
- [ ] SBUS协议
- [ ] 网页配置
