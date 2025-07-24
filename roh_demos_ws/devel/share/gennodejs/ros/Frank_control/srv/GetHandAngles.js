// Auto-generated. Do not edit!

// (in-package Frank_control.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class GetHandAnglesRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetHandAnglesRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetHandAnglesRequest
    let len;
    let data = new GetHandAnglesRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'Frank_control/GetHandAnglesRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetHandAnglesRequest(null);
    return resolved;
    }
};

class GetHandAnglesResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.thumb_bend = null;
      this.index_bend = null;
      this.middle_bend = null;
      this.ring_bend = null;
      this.pinky_bend = null;
      this.thumb_rot = null;
      this.detected = null;
    }
    else {
      if (initObj.hasOwnProperty('thumb_bend')) {
        this.thumb_bend = initObj.thumb_bend
      }
      else {
        this.thumb_bend = 0.0;
      }
      if (initObj.hasOwnProperty('index_bend')) {
        this.index_bend = initObj.index_bend
      }
      else {
        this.index_bend = 0.0;
      }
      if (initObj.hasOwnProperty('middle_bend')) {
        this.middle_bend = initObj.middle_bend
      }
      else {
        this.middle_bend = 0.0;
      }
      if (initObj.hasOwnProperty('ring_bend')) {
        this.ring_bend = initObj.ring_bend
      }
      else {
        this.ring_bend = 0.0;
      }
      if (initObj.hasOwnProperty('pinky_bend')) {
        this.pinky_bend = initObj.pinky_bend
      }
      else {
        this.pinky_bend = 0.0;
      }
      if (initObj.hasOwnProperty('thumb_rot')) {
        this.thumb_rot = initObj.thumb_rot
      }
      else {
        this.thumb_rot = 0.0;
      }
      if (initObj.hasOwnProperty('detected')) {
        this.detected = initObj.detected
      }
      else {
        this.detected = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetHandAnglesResponse
    // Serialize message field [thumb_bend]
    bufferOffset = _serializer.float32(obj.thumb_bend, buffer, bufferOffset);
    // Serialize message field [index_bend]
    bufferOffset = _serializer.float32(obj.index_bend, buffer, bufferOffset);
    // Serialize message field [middle_bend]
    bufferOffset = _serializer.float32(obj.middle_bend, buffer, bufferOffset);
    // Serialize message field [ring_bend]
    bufferOffset = _serializer.float32(obj.ring_bend, buffer, bufferOffset);
    // Serialize message field [pinky_bend]
    bufferOffset = _serializer.float32(obj.pinky_bend, buffer, bufferOffset);
    // Serialize message field [thumb_rot]
    bufferOffset = _serializer.float32(obj.thumb_rot, buffer, bufferOffset);
    // Serialize message field [detected]
    bufferOffset = _serializer.bool(obj.detected, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetHandAnglesResponse
    let len;
    let data = new GetHandAnglesResponse(null);
    // Deserialize message field [thumb_bend]
    data.thumb_bend = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [index_bend]
    data.index_bend = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [middle_bend]
    data.middle_bend = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ring_bend]
    data.ring_bend = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pinky_bend]
    data.pinky_bend = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [thumb_rot]
    data.thumb_rot = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [detected]
    data.detected = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 25;
  }

  static datatype() {
    // Returns string type for a service object
    return 'Frank_control/GetHandAnglesResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ae6e268c4bfe9632c21995915fa1b5ce';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 thumb_bend
    float32 index_bend
    float32 middle_bend
    float32 ring_bend
    float32 pinky_bend
    float32 thumb_rot
    bool detected 
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetHandAnglesResponse(null);
    if (msg.thumb_bend !== undefined) {
      resolved.thumb_bend = msg.thumb_bend;
    }
    else {
      resolved.thumb_bend = 0.0
    }

    if (msg.index_bend !== undefined) {
      resolved.index_bend = msg.index_bend;
    }
    else {
      resolved.index_bend = 0.0
    }

    if (msg.middle_bend !== undefined) {
      resolved.middle_bend = msg.middle_bend;
    }
    else {
      resolved.middle_bend = 0.0
    }

    if (msg.ring_bend !== undefined) {
      resolved.ring_bend = msg.ring_bend;
    }
    else {
      resolved.ring_bend = 0.0
    }

    if (msg.pinky_bend !== undefined) {
      resolved.pinky_bend = msg.pinky_bend;
    }
    else {
      resolved.pinky_bend = 0.0
    }

    if (msg.thumb_rot !== undefined) {
      resolved.thumb_rot = msg.thumb_rot;
    }
    else {
      resolved.thumb_rot = 0.0
    }

    if (msg.detected !== undefined) {
      resolved.detected = msg.detected;
    }
    else {
      resolved.detected = false
    }

    return resolved;
    }
};

module.exports = {
  Request: GetHandAnglesRequest,
  Response: GetHandAnglesResponse,
  md5sum() { return 'ae6e268c4bfe9632c21995915fa1b5ce'; },
  datatype() { return 'Frank_control/GetHandAngles'; }
};
