#ifndef __SLAVEINFO_H__
#define __SLAVEINFO_H__


int si_PDOassign(uint16 slave, uint16 PDOassign, int mapoffset, int bitoffset);
int si_map_sdo(int slave);
int si_siiPDO(uint16 slave, uint8 t, int mapoffset, int bitoffset);
int si_map_sii(int slave);
void si_sdo(int cnt);

void slaveinfo(char *ifname);
#endif /* __SLAVEINFO_H__ */
