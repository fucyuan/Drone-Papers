// Auto-generated. Do not edit!

// (in-package quadrotor_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class SimDebug {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.pos_x = null;
      this.pos_y = null;
      this.pos_z = null;
      this.vel_x = null;
      this.vel_y = null;
      this.vel_z = null;
      this.pwm_1 = null;
      this.pwm_2 = null;
      this.pwm_3 = null;
      this.pwm_4 = null;
      this.omega_x = null;
      this.omega_y = null;
      this.omega_z = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('pos_x')) {
        this.pos_x = initObj.pos_x
      }
      else {
        this.pos_x = 0.0;
      }
      if (initObj.hasOwnProperty('pos_y')) {
        this.pos_y = initObj.pos_y
      }
      else {
        this.pos_y = 0.0;
      }
      if (initObj.hasOwnProperty('pos_z')) {
        this.pos_z = initObj.pos_z
      }
      else {
        this.pos_z = 0.0;
      }
      if (initObj.hasOwnProperty('vel_x')) {
        this.vel_x = initObj.vel_x
      }
      else {
        this.vel_x = 0.0;
      }
      if (initObj.hasOwnProperty('vel_y')) {
        this.vel_y = initObj.vel_y
      }
      else {
        this.vel_y = 0.0;
      }
      if (initObj.hasOwnProperty('vel_z')) {
        this.vel_z = initObj.vel_z
      }
      else {
        this.vel_z = 0.0;
      }
      if (initObj.hasOwnProperty('pwm_1')) {
        this.pwm_1 = initObj.pwm_1
      }
      else {
        this.pwm_1 = 0.0;
      }
      if (initObj.hasOwnProperty('pwm_2')) {
        this.pwm_2 = initObj.pwm_2
      }
      else {
        this.pwm_2 = 0.0;
      }
      if (initObj.hasOwnProperty('pwm_3')) {
        this.pwm_3 = initObj.pwm_3
      }
      else {
        this.pwm_3 = 0.0;
      }
      if (initObj.hasOwnProperty('pwm_4')) {
        this.pwm_4 = initObj.pwm_4
      }
      else {
        this.pwm_4 = 0.0;
      }
      if (initObj.hasOwnProperty('omega_x')) {
        this.omega_x = initObj.omega_x
      }
      else {
        this.omega_x = 0.0;
      }
      if (initObj.hasOwnProperty('omega_y')) {
        this.omega_y = initObj.omega_y
      }
      else {
        this.omega_y = 0.0;
      }
      if (initObj.hasOwnProperty('omega_z')) {
        this.omega_z = initObj.omega_z
      }
      else {
        this.omega_z = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SimDebug
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [pos_x]
    bufferOffset = _serializer.float64(obj.pos_x, buffer, bufferOffset);
    // Serialize message field [pos_y]
    bufferOffset = _serializer.float64(obj.pos_y, buffer, bufferOffset);
    // Serialize message field [pos_z]
    bufferOffset = _serializer.float64(obj.pos_z, buffer, bufferOffset);
    // Serialize message field [vel_x]
    bufferOffset = _serializer.float64(obj.vel_x, buffer, bufferOffset);
    // Serialize message field [vel_y]
    bufferOffset = _serializer.float64(obj.vel_y, buffer, bufferOffset);
    // Serialize message field [vel_z]
    bufferOffset = _serializer.float64(obj.vel_z, buffer, bufferOffset);
    // Serialize message field [pwm_1]
    bufferOffset = _serializer.float64(obj.pwm_1, buffer, bufferOffset);
    // Serialize message field [pwm_2]
    bufferOffset = _serializer.float64(obj.pwm_2, buffer, bufferOffset);
    // Serialize message field [pwm_3]
    bufferOffset = _serializer.float64(obj.pwm_3, buffer, bufferOffset);
    // Serialize message field [pwm_4]
    bufferOffset = _serializer.float64(obj.pwm_4, buffer, bufferOffset);
    // Serialize message field [omega_x]
    bufferOffset = _serializer.float64(obj.omega_x, buffer, bufferOffset);
    // Serialize message field [omega_y]
    bufferOffset = _serializer.float64(obj.omega_y, buffer, bufferOffset);
    // Serialize message field [omega_z]
    bufferOffset = _serializer.float64(obj.omega_z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SimDebug
    let len;
    let data = new SimDebug(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [pos_x]
    data.pos_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pos_y]
    data.pos_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pos_z]
    data.pos_z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [vel_x]
    data.vel_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [vel_y]
    data.vel_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [vel_z]
    data.vel_z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pwm_1]
    data.pwm_1 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pwm_2]
    data.pwm_2 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pwm_3]
    data.pwm_3 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pwm_4]
    data.pwm_4 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [omega_x]
    data.omega_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [omega_y]
    data.omega_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [omega_z]
    data.omega_z = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 104;
  }

  static datatype() {
    // Returns string type for a message object
    return 'quadrotor_msgs/SimDebug';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a8198ed55d087e47fb2498d93f1e3409';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    float64 pos_x
    float64 pos_y
    float64 pos_z
    
    float64 vel_x
    float64 vel_y
    float64 vel_z
    
    float64 pwm_1
    float64 pwm_2
    float64 pwm_3
    float64 pwm_4
    
    float64 omega_x
    float64 omega_y
    float64 omega_z
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SimDebug(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.pos_x !== undefined) {
      resolved.pos_x = msg.pos_x;
    }
    else {
      resolved.pos_x = 0.0
    }

    if (msg.pos_y !== undefined) {
      resolved.pos_y = msg.pos_y;
    }
    else {
      resolved.pos_y = 0.0
    }

    if (msg.pos_z !== undefined) {
      resolved.pos_z = msg.pos_z;
    }
    else {
      resolved.pos_z = 0.0
    }

    if (msg.vel_x !== undefined) {
      resolved.vel_x = msg.vel_x;
    }
    else {
      resolved.vel_x = 0.0
    }

    if (msg.vel_y !== undefined) {
      resolved.vel_y = msg.vel_y;
    }
    else {
      resolved.vel_y = 0.0
    }

    if (msg.vel_z !== undefined) {
      resolved.vel_z = msg.vel_z;
    }
    else {
      resolved.vel_z = 0.0
    }

    if (msg.pwm_1 !== undefined) {
      resolved.pwm_1 = msg.pwm_1;
    }
    else {
      resolved.pwm_1 = 0.0
    }

    if (msg.pwm_2 !== undefined) {
      resolved.pwm_2 = msg.pwm_2;
    }
    else {
      resolved.pwm_2 = 0.0
    }

    if (msg.pwm_3 !== undefined) {
      resolved.pwm_3 = msg.pwm_3;
    }
    else {
      resolved.pwm_3 = 0.0
    }

    if (msg.pwm_4 !== undefined) {
      resolved.pwm_4 = msg.pwm_4;
    }
    else {
      resolved.pwm_4 = 0.0
    }

    if (msg.omega_x !== undefined) {
      resolved.omega_x = msg.omega_x;
    }
    else {
      resolved.omega_x = 0.0
    }

    if (msg.omega_y !== undefined) {
      resolved.omega_y = msg.omega_y;
    }
    else {
      resolved.omega_y = 0.0
    }

    if (msg.omega_z !== undefined) {
      resolved.omega_z = msg.omega_z;
    }
    else {
      resolved.omega_z = 0.0
    }

    return resolved;
    }
};

module.exports = SimDebug;
