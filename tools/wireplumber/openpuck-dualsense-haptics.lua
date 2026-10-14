-- OpenPuck DualSense mode: start the haptic channels of the controller's audio output at full volume.
--
-- Optional (registered by 60-openpuck-dualsense.conf). DualSense audio haptics work without it; it works around
-- a WirePlumber default. WirePlumber starts every output route it has not seen before at
-- device.routes.default-sink-volume (0.064, 40% on the slider). The DualSense UCM "Direct" profile has no
-- hardware volume control, so PipeWire scales the samples themselves and the actuators get ~6% of the haptic
-- signal until the output is raised -- once per device name, and the puck presents several.
--
-- The Direct output carries speaker/headphone audio on channels 1-2 (FL/FR) and the two haptic actuators on
-- channels 3-4 (RL/RR). Only the haptic channels start at full volume; the speaker/headphone channels keep the
-- normal cautious default, so a real DualSense's headphone jack is not affected. Inputs (the mic), the split
-- profile's Speaker/Headphones routes and any volume WirePlumber has already saved are left alone.

devinfo = require ("device-info-cache")
log = Log.open_topic ("s-openpuck")

SimpleEventHook {
  name = "openpuck/dualsense-haptic-volume",
  after = { "device/find-stored-routes", "device/find-best-routes",
            "device/apply-route-props" },
  before = "device/apply-routes",
  interests = {
    EventInterest {
      Constraint { "event.type", "=", "select-routes" },
    },
  },
  execute = function (event)
    local device = event:get_subject ()
    local dp = device.properties
    if dp ["device.vendor.id"] ~= "0x054c" or dp ["device.product.id"] ~= "0x0ce6" then
      return
    end

    local selected_routes = event:get_data ("selected-routes")
    local dev_info = devinfo:get_device_info (device)
    if not selected_routes or not dev_info then
      return
    end

    local default = tonumber (dp ["device.routes.default-sink-volume"])
        or Settings.get_float ("device.routes.default-sink-volume")
    local new_routes = {}
    for device_id, route_json in pairs (selected_routes) do
      new_routes [device_id] = route_json
      local route = Json.Raw (route_json):parse ()
      local ri = devinfo.find_route_info (dev_info, route, false)
      local props = route.props or {}
      -- a saved route arrives with its volumes; only a first-seen Direct output is changed
      if ri and ri.direction == "Output" and ri.name == "[Out] Direct"
          and not props.channelVolumes then
        props.channelVolumes = Json.Array { default, default, 1.0, 1.0 }
        props.channelMap = Json.Array { "FL", "FR", "RL", "RR" }
        new_routes [device_id] = Json.Object {
          index = route.index,
          props = Json.Object (props),
        }:to_string ()
        log:info (device, "haptic channels RL/RR start at full volume on " .. dev_info.name)
      end
    end
    event:set_data ("selected-routes", new_routes)
  end
}:register ()
