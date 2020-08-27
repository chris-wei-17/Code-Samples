// Auto-generated. Do not edit!

// (in-package simulator.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class CarControlData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cv_speed = null;
      this.cv_turn_zero = null;
      this.cv_turn_speed = null;
      this.vp_speed = null;
      this.vp_turn_zero = null;
      this.vp_turn_speed = null;
      this.vesc_speed = null;
    }
    else {
      if (initObj.hasOwnProperty('cv_speed')) {
        this.cv_speed = initObj.cv_speed
      }
      else {
        this.cv_speed = 0.0;
      }
      if (initObj.hasOwnProperty('cv_turn_zero')) {
        this.cv_turn_zero = initObj.cv_turn_zero
      }
      else {
        this.cv_turn_zero = 0.0;
      }
      if (initObj.hasOwnProperty('cv_turn_speed')) {
        this.cv_turn_speed = initObj.cv_turn_speed
      }
      else {
        this.cv_turn_speed = 0.0;
      }
      if (initObj.hasOwnProperty('vp_speed')) {
        this.vp_speed = initObj.vp_speed
      }
      else {
        this.vp_speed = 0;
      }
      if (initObj.hasOwnProperty('vp_turn_zero')) {
        this.vp_turn_zero = initObj.vp_turn_zero
      }
      else {
        this.vp_turn_zero = 0;
      }
      if (initObj.hasOwnProperty('vp_turn_speed')) {
        this.vp_turn_speed = initObj.vp_turn_speed
      }
      else {
        this.vp_turn_speed = 0;
      }
      if (initObj.hasOwnProperty('vesc_speed')) {
        this.vesc_speed = initObj.vesc_speed
      }
      else {
        this.vesc_speed = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CarControlData
    // Serialize message field [cv_speed]
    bufferOffset = _serializer.float64(obj.cv_speed, buffer, bufferOffset);
    // Serialize message field [cv_turn_zero]
    bufferOffset = _serializer.float64(obj.cv_turn_zero, buffer, bufferOffset);
    // Serialize message field [cv_turn_speed]
    bufferOffset = _serializer.float64(obj.cv_turn_speed, buffer, bufferOffset);
    // Serialize message field [vp_speed]
    bufferOffset = _serializer.int16(obj.vp_speed, buffer, bufferOffset);
    // Serialize message field [vp_turn_zero]
    bufferOffset = _serializer.int16(obj.vp_turn_zero, buffer, bufferOffset);
    // Serialize message field [vp_turn_speed]
    bufferOffset = _serializer.int16(obj.vp_turn_speed, buffer, bufferOffset);
    // Serialize message field [vesc_speed]
    bufferOffset = _serializer.float64(obj.vesc_speed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CarControlData
    let len;
    let data = new CarControlData(null);
    // Deserialize message field [cv_speed]
    data.cv_speed = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [cv_turn_zero]
    data.cv_turn_zero = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [cv_turn_speed]
    data.cv_turn_speed = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [vp_speed]
    data.vp_speed = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [vp_turn_zero]
    data.vp_turn_zero = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [vp_turn_speed]
    data.vp_turn_speed = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [vesc_speed]
    data.vesc_speed = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 38;
  }

  static datatype() {
    // Returns string type for a message object
    return 'simulator/CarControlData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '324d1d835c12674bf092534c6d2533e0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Constants from controlling the car via node
    
    # cmd_vel
    float64 cv_speed
    
    float64 cv_turn_zero
    float64 cv_turn_speed
    
    # drive_pwm
    int16 vp_speed
    
    int16 vp_turn_zero
    int16 vp_turn_speed
    
    # VESC
    float64 vesc_speed
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CarControlData(null);
    if (msg.cv_speed !== undefined) {
      resolved.cv_speed = msg.cv_speed;
    }
    else {
      resolved.cv_speed = 0.0
    }

    if (msg.cv_turn_zero !== undefined) {
      resolved.cv_turn_zero = msg.cv_turn_zero;
    }
    else {
      resolved.cv_turn_zero = 0.0
    }

    if (msg.cv_turn_speed !== undefined) {
      resolved.cv_turn_speed = msg.cv_turn_speed;
    }
    else {
      resolved.cv_turn_speed = 0.0
    }

    if (msg.vp_speed !== undefined) {
      resolved.vp_speed = msg.vp_speed;
    }
    else {
      resolved.vp_speed = 0
    }

    if (msg.vp_turn_zero !== undefined) {
      resolved.vp_turn_zero = msg.vp_turn_zero;
    }
    else {
      resolved.vp_turn_zero = 0
    }

    if (msg.vp_turn_speed !== undefined) {
      resolved.vp_turn_speed = msg.vp_turn_speed;
    }
    else {
      resolved.vp_turn_speed = 0
    }

    if (msg.vesc_speed !== undefined) {
      resolved.vesc_speed = msg.vesc_speed;
    }
    else {
      resolved.vesc_speed = 0.0
    }

    return resolved;
    }
};

module.exports = CarControlData;
