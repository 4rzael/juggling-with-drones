'use strict';

// etablit une connexion temps réel
var socket = io();

var gyroData = undefined
var compassData = undefined

function debug() {
  socket.emit('debug', arguments)
}

$(document).ready(function () {

  $(window).on('error', function (error) {
    socket.emit('error', error)
    return false
  })

  // s'il y a un gyroscope/accelerometre
  if (window.DeviceMotionEvent !== undefined) {
    window.ondevicemotion = function(e) {
    // quand le portable bouge :
      gyroData = e;
    };
  }

  if (window.DeviceOrientationEvent) {
    // Listen for the deviceorientation event and handle the raw data
    window.ondeviceorientation = function(e) {
      if(e.webkitCompassHeading) {
        // Apple works only with this, alpha doesn't work
        compassData = e.webkitCompassHeading;  
      }
      else compassData = e.alpha;
    };
  }

  var big_button_is_pressed = false
  var abort_button_is_pressed = false
  var traj_buttons = ['#traj0_button', '#traj1_button', '#traj2_button', '#traj3_button', '#start_stop_button', '#slide_button']
  var traj_buttons_pressed = {}

  traj_buttons.forEach(function(button) {
    traj_buttons_pressed[button] = false
    $(button)
    .on('touchstart', function () {
      traj_buttons_pressed[button] = true
    })
    .on('touchend', function () {
      traj_buttons_pressed[button] = false
    })
  })

  var acc
  var gyro
  var compass

  setInterval(function () {

    var controller_id = $('input[name=controller_id]:checked').val();
    var drone_id = $('input[name=drone_id]:checked').val();

    var trajectory_type = $('input[name=trajectory_type]:checked').val();
    var big_button = big_button_is_pressed

    if (gyroData) {
      if (gyroData.acceleration) {
        acc = {
          x: gyroData.acceleration.x,
          y: gyroData.acceleration.y,
          z: gyroData.acceleration.z      
        }
      }
      if (gyroData.rotationRate) {
        gyro = {
          alpha: gyroData.rotationRate.alpha,
          beta: gyroData.rotationRate.alpha,
          gamma: gyroData.rotationRate.alpha
        }
      }
      compass = compassData
    }

    socket.emit('glove_input',
      {
        controller_id: controller_id,
        drone_id: drone_id,
        trajectory_selections: traj_buttons.map(function(button) {
          return traj_buttons_pressed[button] ? 1 : 0
        }),
        big_button: big_button_is_pressed ? 1 : 0,
        acc: acc,
        gyro: gyro,
        compass: compass,
        abort: abort_button_is_pressed ? 1 : 0
      });
    gyroData = undefined;
  }, 1000.0 / 120.0);

  $('#big_button')
  .on('touchstart', function () {
    big_button_is_pressed = true
  })
  .on('touchend', function () {
    big_button_is_pressed = false
  })

  $('#abort_button')
  .on('touchstart', function () {
    abort_button_is_pressed = true
  })
  .on('touchend', function () {
    abort_button_is_pressed = false
  })

})