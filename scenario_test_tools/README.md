# Automated testing

The automated tests in `scenario_test_tools` provide a basic framework for 
mocked, scriptable and introspectable ROS services and action servers.
These are operated from the same process so they can offer a correct, integral mock for various aspects of the robot.
This allows to guide the high-level behavior into edge-cases and verify they are handled as intended. 

The core idea is that a tests specifies a sequence of services/actions that the behavior should call.
A test can inspect the request/goal and reply with e.g. success or failure and 
then check if the failure is handled correctly etc. 

# Framework

The core of the framework is the `ScriptableBase`, 
on which `ScriptableActionserver` and `ScriptableServiceServer` are based.

`ScriptableBase` allows to
- introspect past goals via `received_goals`. This is simply an list of all received goals.
    - This can be combined with the `remember_goals` context manager, 
    which makes goals be forgotten before entering the context and after exiting again. 
    Useful when you have multiple tests in sequence. 
- set the next result to a to-be-received goal with the `reply` method.
  There are several similar methods:
  - `reply_conditionally`: which takes a callable that returns True or False based on the goal and sends back the corresponding result
  - `await_goal` + `reply_directly`. `await_goal` blocks and returns a goal once it's received, so the goal can be inspected. 
    Based on that, specify the reply with `reply_directly`.
- Set default results via `default_result` without bothering with `reply*` most of the time, 
    e.g for actions that are expected to always success on the real robot, eg. for doing text-to-speech.
    If you want to override temporarily, use this syntax:
  
  ```python
  with scriptable_thing.custom_reply():
      scriptable_thing.reply(SomeActionResult('non-default'))
  ```

# Examples
The `scripts` directory contains 2 scripts: `example_test.py` and `dummy_behavior.py`. 
The dummy behavior is tested with the example test, which uses a reasonable subset of the features in the framework.