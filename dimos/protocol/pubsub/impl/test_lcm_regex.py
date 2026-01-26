#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for LCM regex subscription support."""

from collections.abc import Generator
import time

import pytest

from dimos.protocol.pubsub.impl.lcmpubsub import Glob, LCMPubSubBase, Topic


@pytest.fixture
def lcm() -> Generator[LCMPubSubBase, None, None]:
    lcm = LCMPubSubBase(autoconf=True)
    lcm.start()
    yield lcm
    lcm.stop()


def test_subscribe_regex_via_topic(lcm: LCMPubSubBase) -> None:
    """Test that regex pattern in Topic matches multiple channels and returns actual topic."""
    import re

    received: list[tuple[bytes, Topic]] = []

    # Use re.compile() to indicate this is a pattern subscription
    pattern_topic = Topic(topic=re.compile(r"/sensor/.*"))
    lcm.subscribe(pattern_topic, lambda msg, topic: received.append((msg, topic)))

    lcm.publish(Topic("/sensor/temp"), b"temp_data")
    lcm.publish(Topic("/sensor/humidity"), b"humidity_data")
    lcm.publish(Topic("/other/topic"), b"should_not_match")

    time.sleep(0.1)

    assert len(received) == 2

    # Check we received the actual matched topics, not the pattern
    topics = {r[1].topic for r in received}
    assert "/sensor/temp" in topics
    assert "/sensor/humidity" in topics

    # Check data
    data = {r[0] for r in received}
    assert b"temp_data" in data
    assert b"humidity_data" in data


def test_subscribe_glob_via_topic(lcm: LCMPubSubBase) -> None:
    """Test that Glob pattern in Topic matches channels using glob syntax."""
    received: list[tuple[bytes, Topic]] = []

    # Use Glob for glob-style pattern matching
    pattern_topic = Topic(topic=Glob("/sensor/*"))
    lcm.subscribe(pattern_topic, lambda msg, topic: received.append((msg, topic)))

    lcm.publish(Topic("/sensor/temp"), b"temp_data")
    lcm.publish(Topic("/sensor/humidity"), b"humidity_data")
    lcm.publish(Topic("/sensor/nested/deep"), b"should_not_match_single_star")
    lcm.publish(Topic("/other/topic"), b"should_not_match")

    time.sleep(0.1)

    assert len(received) == 2
    topics = {r[1].topic for r in received}
    assert "/sensor/temp" in topics
    assert "/sensor/humidity" in topics


def test_subscribe_glob_doublestar(lcm: LCMPubSubBase) -> None:
    """Test that ** in Glob matches nested paths."""
    received: list[tuple[bytes, Topic]] = []

    pattern_topic = Topic(topic=Glob("/robot/**"))
    lcm.subscribe(pattern_topic, lambda msg, topic: received.append((msg, topic)))

    lcm.publish(Topic("/robot/arm"), b"arm")
    lcm.publish(Topic("/robot/arm/joint1"), b"joint1")
    lcm.publish(Topic("/robot/leg/motor/speed"), b"speed")
    lcm.publish(Topic("/sensor/temp"), b"should_not_match")

    time.sleep(0.1)

    assert len(received) == 3
    topics = {r[1].topic for r in received}
    assert "/robot/arm" in topics
    assert "/robot/arm/joint1" in topics
    assert "/robot/leg/motor/speed" in topics
