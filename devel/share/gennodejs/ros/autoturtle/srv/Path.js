// Auto-generated. Do not edit!

// (in-package autoturtle.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class PathRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.start = null;
      this.stop = null;
    }
    else {
      if (initObj.hasOwnProperty('start')) {
        this.start = initObj.start
      }
      else {
        this.start = [];
      }
      if (initObj.hasOwnProperty('stop')) {
        this.stop = initObj.stop
      }
      else {
        this.stop = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PathRequest
    // Serialize message field [start]
    bufferOffset = _arraySerializer.int32(obj.start, buffer, bufferOffset, null);
    // Serialize message field [stop]
    bufferOffset = _arraySerializer.int32(obj.stop, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PathRequest
    let len;
    let data = new PathRequest(null);
    // Deserialize message field [start]
    data.start = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [stop]
    data.stop = _arrayDeserializer.int32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.start.length;
    length += 4 * object.stop.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'autoturtle/PathRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c0568d4421a7f52d03eeb3cde18aa062';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32[] start
    int32[] stop
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PathRequest(null);
    if (msg.start !== undefined) {
      resolved.start = msg.start;
    }
    else {
      resolved.start = []
    }

    if (msg.stop !== undefined) {
      resolved.stop = msg.stop;
    }
    else {
      resolved.stop = []
    }

    return resolved;
    }
};

class PathResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.path = null;
    }
    else {
      if (initObj.hasOwnProperty('path')) {
        this.path = initObj.path
      }
      else {
        this.path = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PathResponse
    // Serialize message field [path]
    bufferOffset = _serializer.string(obj.path, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PathResponse
    let len;
    let data = new PathResponse(null);
    // Deserialize message field [path]
    data.path = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.path);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'autoturtle/PathResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1d00cd540af97efeb6b1589112fab63e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string path
    # int32[] path
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PathResponse(null);
    if (msg.path !== undefined) {
      resolved.path = msg.path;
    }
    else {
      resolved.path = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: PathRequest,
  Response: PathResponse,
  md5sum() { return 'd32b3108e0653dfb8af9c9692bc47b31'; },
  datatype() { return 'autoturtle/Path'; }
};
