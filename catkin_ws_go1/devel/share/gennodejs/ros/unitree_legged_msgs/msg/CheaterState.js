// Auto-generated. Do not edit!

// (in-package unitree_legged_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class CheaterState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.orientation = null;
      this.position = null;
      this.vBody = null;
      this.vWorld = null;
      this.omegaBody = null;
      this.acceleration = null;
    }
    else {
      if (initObj.hasOwnProperty('orientation')) {
        this.orientation = initObj.orientation
      }
      else {
        this.orientation = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('vBody')) {
        this.vBody = initObj.vBody
      }
      else {
        this.vBody = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('vWorld')) {
        this.vWorld = initObj.vWorld
      }
      else {
        this.vWorld = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('omegaBody')) {
        this.omegaBody = initObj.omegaBody
      }
      else {
        this.omegaBody = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('acceleration')) {
        this.acceleration = initObj.acceleration
      }
      else {
        this.acceleration = new Array(3).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CheaterState
    // Check that the constant length array field [orientation] has the right length
    if (obj.orientation.length !== 4) {
      throw new Error('Unable to serialize array field orientation - length must be 4')
    }
    // Serialize message field [orientation]
    bufferOffset = _arraySerializer.float32(obj.orientation, buffer, bufferOffset, 4);
    // Check that the constant length array field [position] has the right length
    if (obj.position.length !== 3) {
      throw new Error('Unable to serialize array field position - length must be 3')
    }
    // Serialize message field [position]
    bufferOffset = _arraySerializer.float32(obj.position, buffer, bufferOffset, 3);
    // Check that the constant length array field [vBody] has the right length
    if (obj.vBody.length !== 3) {
      throw new Error('Unable to serialize array field vBody - length must be 3')
    }
    // Serialize message field [vBody]
    bufferOffset = _arraySerializer.float32(obj.vBody, buffer, bufferOffset, 3);
    // Check that the constant length array field [vWorld] has the right length
    if (obj.vWorld.length !== 3) {
      throw new Error('Unable to serialize array field vWorld - length must be 3')
    }
    // Serialize message field [vWorld]
    bufferOffset = _arraySerializer.float32(obj.vWorld, buffer, bufferOffset, 3);
    // Check that the constant length array field [omegaBody] has the right length
    if (obj.omegaBody.length !== 3) {
      throw new Error('Unable to serialize array field omegaBody - length must be 3')
    }
    // Serialize message field [omegaBody]
    bufferOffset = _arraySerializer.float32(obj.omegaBody, buffer, bufferOffset, 3);
    // Check that the constant length array field [acceleration] has the right length
    if (obj.acceleration.length !== 3) {
      throw new Error('Unable to serialize array field acceleration - length must be 3')
    }
    // Serialize message field [acceleration]
    bufferOffset = _arraySerializer.float32(obj.acceleration, buffer, bufferOffset, 3);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CheaterState
    let len;
    let data = new CheaterState(null);
    // Deserialize message field [orientation]
    data.orientation = _arrayDeserializer.float32(buffer, bufferOffset, 4)
    // Deserialize message field [position]
    data.position = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [vBody]
    data.vBody = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [vWorld]
    data.vWorld = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [omegaBody]
    data.omegaBody = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [acceleration]
    data.acceleration = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    return data;
  }

  static getMessageSize(object) {
    return 76;
  }

  static datatype() {
    // Returns string type for a message object
    return 'unitree_legged_msgs/CheaterState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f8cc1254d33feda52dbc17fc7e4c7459';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[4] orientation
    float32[3] position
    float32[3] vBody
    float32[3] vWorld
    float32[3] omegaBody
    float32[3] acceleration
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CheaterState(null);
    if (msg.orientation !== undefined) {
      resolved.orientation = msg.orientation;
    }
    else {
      resolved.orientation = new Array(4).fill(0)
    }

    if (msg.position !== undefined) {
      resolved.position = msg.position;
    }
    else {
      resolved.position = new Array(3).fill(0)
    }

    if (msg.vBody !== undefined) {
      resolved.vBody = msg.vBody;
    }
    else {
      resolved.vBody = new Array(3).fill(0)
    }

    if (msg.vWorld !== undefined) {
      resolved.vWorld = msg.vWorld;
    }
    else {
      resolved.vWorld = new Array(3).fill(0)
    }

    if (msg.omegaBody !== undefined) {
      resolved.omegaBody = msg.omegaBody;
    }
    else {
      resolved.omegaBody = new Array(3).fill(0)
    }

    if (msg.acceleration !== undefined) {
      resolved.acceleration = msg.acceleration;
    }
    else {
      resolved.acceleration = new Array(3).fill(0)
    }

    return resolved;
    }
};

module.exports = CheaterState;
