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
The `scripts` directory contains several scripts: 
- `example_test.py` and `dummy_behavior.py` work together.
    The dummy behavior will wait for commands to have a robot charge itself and plug itself in.
    The test sends these commands and checks that the behavior handles this command correctly.
    + Run the dummy_behavior with `rosrun scenario_test_tools dummy_behavior.py`
    + Then start the test with `rosrun scenario_test_tools example_test.py --start_scenario=test_all`

- `move_base_success.py` uses the default_result feature to have a move_base implementation that always succeeds after a given time
- `example_move_base` is a more elaborate version of the above, handling each goal a differently to show other features of the framework.
    First goal received succeeds, 2nd is aborted and 3rd is ignored. Further goals are not expected and not handled at all.
