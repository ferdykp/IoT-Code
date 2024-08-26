void LoRa_setConfig(struct LoRa_config config){
  LoRa.setFrequency(config.Frequency);
  LoRa.setSpreadingFactor(config.SpreadingFactor);
  LoRa.setSignalBandwidth(config.SignalBandwidth);
  LoRa.setCodingRate4(config.CodingRate4);
  if (config.enableCrc)
    LoRa.enableCrc();
  else
    LoRa.disableCrc();
  if (config.invertIQ)
    LoRa.enableInvertIQ();
  else
    LoRa.disableInvertIQ();
  LoRa.setSyncWord(config.SyncWord);
  LoRa.setPreambleLength(config.PreambleLength);
}

void LoRa_TxMode(){
  LoRa_setConfig(txLoRa);
  LoRa.idle();  
}

void LoRa_sendData(){
  LoRa_TxMode();
  LoRaWanPacket.clear();
  LoRaWanPacket.print(myStr);
  
  if (LoRaWanPacket.encode()){
    LoRa.beginPacket();
    LoRa.write(LoRaWanPacket.buffer(), LoRaWanPacket.length());
    LoRa.endPacket();
  }
}
