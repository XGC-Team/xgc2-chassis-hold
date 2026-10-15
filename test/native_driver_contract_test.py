#!/usr/bin/env python3
"""Lexically inventory reviewed native driver call sites.

This checks tokens, known writer locations and textual order. It is not a C++
control-flow analysis: native driver runtime tests establish execution. The
installed CTest invocation repeats this source check, not an installed driver.
"""
import argparse
from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[3] / 'robotics'


def code(text):
    # Keep offsets and line breaks while excluding comments and literals.
    return re.sub(r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\])*"',
                  lambda match: re.sub(r'[^\n]', ' ', match[0]), text, flags=re.S)


def methods(text, owner):
    clean = code(text)
    result = {}
    for match in re.finditer(r'\b' + owner + r'::(~?\w+)\s*\([^;{}]*\)\s*\{', clean):
        depth, end = 1, match.end()
        while depth and end < len(clean):
            depth += (clean[end] == '{') - (clean[end] == '}')
            end += 1
        if depth:
            raise AssertionError('unbalanced driver method')
        result[match[1]] = clean[match.end():end - 1]
    return result


def calls(text, name):
    return list(re.finditer(r'\b' + re.escape(name) + r'\s*\(', text))


def validate(text, owner, callback, writer, allowed_writers, loop):
    bodies = methods(text, owner)
    body = bodies[callback]
    if not re.search(r'hold_\s*&&\s*!hold_->admit\(hold_robot_id_,\s*event.getReceiptTime\(\).toNSec\(\)\)\)\s*\{?\s*(?:ROS_DEBUG\([^;]*;\s*)?return;', body):
        raise AssertionError('expected ROS receipt-stamp rejection text missing')
    outputs = calls(body, writer)
    if not outputs:
        raise AssertionError('motion callback writer token missing')
    if body.index('return;') > outputs[0].start():
        raise AssertionError('motion output text precedes admission text')
    actual = {name for name, value in bodies.items() if calls(value, writer)}
    if actual != allowed_writers:
        raise AssertionError(f'unreviewed motion writers: {actual ^ allowed_writers}')
    if not re.search(r'hold_\s*->\s*tick\s*\(\s*\)', bodies[loop]):
        raise AssertionError('native control loop text must include a HOLD tick')
    # Flag these simple, explicit disabled-code mutations. This deliberately
    # does not attempt to decide arbitrary C++ reachability or branch semantics.
    for name in (callback, loop):
        if re.search(r'\bif\s*\(\s*(?:false|0)\s*\)', bodies[name]):
            raise AssertionError('literal false guard in an audited method requires review')
    if 'UdpEndpoint' not in code(text) or 'hold_->service()' not in code(text):
        raise AssertionError('expected own-HOLD endpoint tokens missing')
    if not re.search(r'options.domain.clock\s*=\s*\[\]\s*\{\s*return ros::Time::now\(\).toNSec\(\);', text):
        raise AssertionError('expected matching ROS clock assignment text missing')
    # Count call tokens even when normal whitespace precedes the parenthesis.
    definitions = len(re.findall(owner + r'\s*::\s*' + re.escape(writer) + r'\s*\(', code(text)))
    if len(calls(code(text), writer)) - definitions != sum(len(calls(value, writer)) for value in bodies.values()):
        raise AssertionError('motion output outside inventoried driver methods')


class NativeDriverContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.wheeltec = (ROOT / 'wheeltec/onboard/ros1/chassis/src/turn_on_wheeltec_robot/src/wheeltec_robot.cpp').read_text()
        cls.scout = (ROOT / 'agilex/onboard/ros1/chassis/src/scout_base/src/scout_messenger.cpp').read_text()

    def wheel(self, text):
        validate(text, 'turn_on_robot', 'Cmd_Vel_Callback', 'WriteMotion',
                 {'Cmd_Vel_Callback', 'write_zero', 'turn_on_robot', '~turn_on_robot'}, 'Control')
        bodies = methods(text, 'turn_on_robot')
        self.assertEqual({name for name, body in bodies.items()
                          if re.search(r'\bStm32_Serial\s*\.\s*write\s*\(', body)}, {'WriteMotion'})
        for name in ('write_zero', 'turn_on_robot', '~turn_on_robot'):
            arguments = re.findall(r'\bWriteMotion\s*\(([^;]*?)\)', bodies[name])
            self.assertTrue(arguments)
            self.assertEqual({re.sub(r'\s+', '', value) for value in arguments}, {'geometry_msgs::Twist{}'})

    def agilex(self, text):
        validate(text, 'ScoutROSMessenger', 'TwistCmdCallback', 'SetMotionCommand',
                 {'TwistCmdCallback', 'HoldZero'}, 'ControlTick')
        bodies = methods(text, 'ScoutROSMessenger')
        self.assertRegex(bodies['HoldZero'], r'\bSetMotionCommand\s*\(\s*0\.0\s*,\s*0\.0\s*\)')
        self.assertRegex(bodies['write_zero'], r'\bHoldZero\s*\(\s*\)')
        # Textual order only; the native runtime test exercises this branch.
        self.assertLess(bodies['TwistCmdCallback'].index('return;'), bodies['TwistCmdCallback'].index('current_twist_ ='))

    def test_wheeltec_lexical_callsite_inventory(self):
        self.wheel(self.wheeltec)

    def test_scout_lexical_callsite_inventory(self):
        self.agilex(self.scout)

    def test_dispatch_time_cannot_replace_receipt_time(self):
        for check, text in ((self.wheel, self.wheeltec), (self.agilex, self.scout)):
            with self.assertRaises(AssertionError):
                check(text.replace('event.getReceiptTime().toNSec()', 'ros::Time::now().toNSec()'))

    def test_new_motion_writer_is_flagged(self):
        for check, text, owner, writer in (
                (self.wheel, self.wheeltec, 'turn_on_robot', 'WriteMotion(twist)'),
                (self.agilex, self.scout, 'ScoutROSMessenger', 'scout_->SetMotionCommand(1, 1)')):
            with self.assertRaises(AssertionError):
                check(text + f'\nvoid {owner}::NewInput() {{ {writer}; }}\n')

    def test_spaced_call_tokens_are_inventoried(self):
        for check, text, owner, writer in (
                (self.wheel, self.wheeltec, 'turn_on_robot', 'WriteMotion (twist)'),
                (self.agilex, self.scout, 'ScoutROSMessenger', 'scout_->SetMotionCommand \n(1, 1)')):
            with self.assertRaises(AssertionError):
                check(text + f'\nvoid {owner}::NewInput() {{ {writer}; }}\n')

    def test_reviewed_native_calls_allow_spacing_before_parentheses(self):
        self.wheel(re.sub(r'\bWriteMotion\s*\(', 'WriteMotion \n(', self.wheeltec))
        self.agilex(re.sub(r'\bSetMotionCommand\s*\(', 'SetMotionCommand \n(', self.scout))

    def test_literal_false_admission_wrapper_is_flagged(self):
        for check, text in ((self.wheel, self.wheeltec), (self.agilex, self.scout)):
            with self.assertRaises(AssertionError):
                check(text.replace('if (hold_ && !hold_->admit',
                                   'if (false) if (hold_ && !hold_->admit', 1))

    def test_literal_false_tick_wrapper_is_flagged(self):
        for check, text in ((self.wheel, self.wheeltec), (self.agilex, self.scout)):
            with self.assertRaises(AssertionError):
                check(text.replace('hold_->tick()', 'if (false) hold_->tick()'))

    def test_native_loop_cannot_lose_the_hold_tick(self):
        for check, text in ((self.wheel, self.wheeltec), (self.agilex, self.scout)):
            with self.assertRaises(AssertionError):
                check(text.replace('hold_->tick()', 'unrelated_tick()'))

    def test_native_output_cannot_precede_the_gate(self):
        for check, text, writer in ((self.wheel, self.wheeltec, 'WriteMotion(twist);'),
                                    (self.agilex, self.scout, 'scout_->SetMotionCommand(1, 1);')):
            with self.assertRaises(AssertionError):
                check(text.replace('if (hold_ && !hold_->admit', writer + '\nif (hold_ && !hold_->admit'))

    def test_driver_cannot_expose_an_unrelated_service(self):
        for check, text in ((self.wheel, self.wheeltec), (self.agilex, self.scout)):
            with self.assertRaises(AssertionError):
                check(text.replace('hold_->service()', 'unrelated_service()'))

    def test_clock_cannot_differ_from_the_receipt_clock(self):
        for check, text in ((self.wheel, self.wheeltec), (self.agilex, self.scout)):
            with self.assertRaises(AssertionError):
                check(text.replace('options.domain.clock = [] { return ros::Time::now().toNSec(); };',
                                   'options.domain.clock = unrelated_clock;'))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--robotics-root', type=Path, default=ROOT)
    args, remaining = parser.parse_known_args()
    ROOT = args.robotics_root
    unittest.main(argv=[__file__, *remaining])
