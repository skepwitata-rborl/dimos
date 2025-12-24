from typing import Optional
from pydantic import BaseModel, Field


class ButtonState(BaseModel):
    value: float = Field(ge=0.0, le=1.0, description="Analog value (0-1)")
    pressed: bool = Field(description="Whether button is pressed")
    touched: bool = Field(description="Whether button is touched")


class ControllerButtons(BaseModel):
    trigger: ButtonState = Field(description="Index trigger (analog)")
    grip: ButtonState = Field(description="Grip/squeeze button")
    menu: ButtonState = Field(description="Menu/system button")
    thumbstick: ButtonState = Field(description="Thumbstick click")
    x_or_a: ButtonState = Field(description="X button (left) or A button (right)")
    y_or_b: ButtonState = Field(description="Y button (left) or B button (right)")


class ControllerAxes(BaseModel):
    thumbstick_x: float = Field(ge=-1.0, le=1.0, description="Thumbstick X-axis")
    thumbstick_y: float = Field(ge=-1.0, le=1.0, description="Thumbstick Y-axis")


class ControllerData(BaseModel):
    connected: bool = Field(description="Whether controller is connected")
    position: tuple[float, float, float] = Field(description="Position (x, y, z) in meters")
    rotation: tuple[float, float, float, float] = Field(
        description="Rotation quaternion (x, y, z, w)"
    )
    rotation_euler: tuple[float, float, float] = Field(
        description="Rotation euler angles (x, y, z) in degrees"
    )
    buttons: ControllerButtons = Field(description="Button states")
    button_values: list[float] = Field(description="Raw button values array")
    axes: ControllerAxes = Field(description="Thumbstick axes")


class ControllerFrame(BaseModel):
    timestamp: float = Field(description="Timestamp in seconds")
    left: Optional[ControllerData] = Field(None, description="Left controller data")
    right: Optional[ControllerData] = Field(None, description="Right controller data")
