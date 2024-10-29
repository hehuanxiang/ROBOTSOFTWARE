import unittest
from unittest.mock import MagicMock, patch

# 模拟 Stepper 类
class Stepper:
    def __init__(self, pins):
        self.pins = pins

    def step(self, steps, direction, speed, docking):
        return None  # 返回一个模拟的操作结果

# 被测试的函数
def handle_stop(sign):
    if sign is not None:
        testStepper = Stepper([13, 15, 11, 40, 37, 38])

        if sign == "docked":
            print("docked2")
            action = testStepper.step(3000, "left", 5, docking=False)
            handle_stop(action)

        elif sign == "right_end":
            print("end3")
            action = testStepper.step(10000000, "right", 50, docking=True)
            handle_stop(action)

# 单元测试类
class TestHandleStop(unittest.TestCase):
    @patch('__main__.Stepper')  # 使用 mock 替换 Stepper 类
    def test_handle_stop_docked(self, MockStepper):
        # 模拟 step 方法的行为
        mock_stepper_instance = MockStepper.return_value
        mock_stepper_instance.step.return_value = None

        handle_stop("docked")

        # 验证 step 方法是否被正确调用
        mock_stepper_instance.step.assert_called_with(3000, "left", 5, docking=False)

    @patch('__main__.Stepper')  # 使用 mock 替换 Stepper 类
    def test_handle_stop_right_end(self, MockStepper):
        mock_stepper_instance = MockStepper.return_value
        mock_stepper_instance.step.return_value = None

        handle_stop("right_end")

        # 验证 step 方法是否被正确调用
        mock_stepper_instance.step.assert_called_with(10000000, "right", 50, docking=True)

    def test_handle_stop_none(self):
        # 测试当 sign 为 None 时，不会抛出异常
        try:
            handle_stop(None)
        except Exception as e:
            self.fail(f"handle_stop(None) 发生异常: {e}")

# 运行测试
if __name__ == '__main__':
    unittest.main()
