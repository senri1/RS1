
"use strict";

let DeleteSoundSource = require('./DeleteSoundSource.js')
let AddSoundSource = require('./AddSoundSource.js')
let LoadExternalMap = require('./LoadExternalMap.js')
let LoadMap = require('./LoadMap.js')
let AddThermalSource = require('./AddThermalSource.js')
let DeleteRfidTag = require('./DeleteRfidTag.js')
let RegisterGui = require('./RegisterGui.js')
let AddCO2Source = require('./AddCO2Source.js')
let DeleteThermalSource = require('./DeleteThermalSource.js')
let DeleteCO2Source = require('./DeleteCO2Source.js')
let MoveRobot = require('./MoveRobot.js')
let AddRfidTag = require('./AddRfidTag.js')

module.exports = {
  DeleteSoundSource: DeleteSoundSource,
  AddSoundSource: AddSoundSource,
  LoadExternalMap: LoadExternalMap,
  LoadMap: LoadMap,
  AddThermalSource: AddThermalSource,
  DeleteRfidTag: DeleteRfidTag,
  RegisterGui: RegisterGui,
  AddCO2Source: AddCO2Source,
  DeleteThermalSource: DeleteThermalSource,
  DeleteCO2Source: DeleteCO2Source,
  MoveRobot: MoveRobot,
  AddRfidTag: AddRfidTag,
};
