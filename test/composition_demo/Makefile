SHELL := /bin/bash

.PHONY: help test-all test-record test-replay test-replay-standalone test-verify clean

help:
	@echo "ROS Composition Demo Test"
	@echo ""
	@echo "Targets:"
	@echo "  test-all              - Run all tests (record + replay + verify)"
	@echo "  test-record           - Record composition_demo.launch.py"
	@echo "  test-replay           - Replay with service-based loading"
	@echo "  test-replay-standalone - Replay in standalone mode"
	@echo "  test-verify           - Verify the replay worked"
	@echo "  clean                 - Clean generated files"
	@echo ""

test-all: clean test-record test-replay test-verify
	@echo ""
	@echo "✅ All tests completed!"

test-record:
	@echo "📝 Recording composition demo..."
	@. /opt/ros/humble/setup.bash && \
	. ../../install/setup.bash && \
	play_launch dump launch composition composition_demo.launch.py
	@echo "✅ Recording complete: record.json created"
	@echo ""

test-replay:
	@echo "▶️  Replaying with service-based component loading..."
	@. /opt/ros/humble/setup.bash && \
	. ../../install/setup.bash && \
	timeout 5 play_launch replay || true
	@echo "✅ Replay complete (timed out after 5s as expected)"
	@echo ""

test-replay-standalone:
	@echo "▶️  Replaying in standalone mode..."
	@rm -rf play_log_standalone record_standalone.json
	@cp record.json record_standalone.json
	@. /opt/ros/humble/setup.bash && \
	. ../../install/setup.bash && \
	timeout 5 play_launch replay \
		--config record_standalone.json \
		--standalone-composable-nodes || true
	@echo "✅ Standalone replay complete"
	@echo ""

test-verify:
	@echo "🔍 Verifying replay results..."
	@./scripts/verify_replay.sh

clean:
	@echo "🧹 Cleaning up..."
	@rm -rf record.json record_standalone.json play_log play_log_standalone
	@echo "✅ Cleanup complete"
