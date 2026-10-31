import json
TAB="pm_tab"; BROKER="pm_broker"; UITAB="pm_ui_tab"
G_RAIL,G_MODE,G_LOAD,G_SIM="pm_g_rail","pm_g_mode","pm_g_load","pm_g_sim"
js=lambda n: open("js/"+n).read()
nodes=[]
def add(d): nodes.append(d); return d["id"]

add({"id":TAB,"type":"tab","label":"Power Management (Issue #10)","disabled":False,
     "info":"Battery / 5 V rail meter, power-mode toggle (Active / Sleep / Emergency Shutdown) and relay load view for the Water Rover v2 power manager.\n\nMQTT topics:\n- rover/power/status (in)  JSON from the ESP32\n- rover/power/cmd (out)    active | sleep | emergency | reset\n- rover/power/req/1..4 (out) 1 | 0  load requests\n\nNo hardware? Switch on 'Simulate ESP32' on the dashboard."})
add({"id":BROKER,"type":"mqtt-broker","name":"Local","broker":"localhost","port":"1883","clientid":"","autoConnect":True,"usetls":False,
     "protocolVersion":"4","keepalive":"60","cleansession":True,"autoUnsubscribe":True,
     "birthTopic":"","birthQos":"0","birthPayload":"","birthMsg":{},"closeTopic":"","closeQos":"0","closePayload":"","closeMsg":{},"willTopic":"","willQos":"0","willPayload":"","willMsg":{},"userProps":"","sessionExpiry":""})
add({"id":UITAB,"type":"ui_tab","name":"Power","icon":"battery_charging_full","disabled":False,"hidden":False,"order":2})
for gid,name,order,w in [(G_RAIL,"Battery & 5 V rail",1,"6"),(G_MODE,"Power mode",2,"6"),(G_LOAD,"Load channels (relays)",3,"6"),(G_SIM,"Simulation (no hardware)",4,"6")]:
    add({"id":gid,"type":"ui_group","name":name,"tab":UITAB,"order":order,"disp":True,"width":w,"collapse":False,"className":""})

add({"id":"pm_note","type":"comment","z":TAB,"name":"Issue #10 - Power Distribution & Load Management","info":"Status in: rover/power/status. Commands out: rover/power/cmd, rover/power/req/N.","x":230,"y":40,"wires":[]})

# --- inbound status ---
add({"id":"pm_mqtt_in","type":"mqtt in","z":TAB,"name":"rover/power/status","topic":"rover/power/status","qos":"0","datatype":"json","broker":BROKER,"nl":False,"rap":True,"rh":0,"inputs":0,"x":150,"y":120,"wires":[["pm_route"]]})
add({"id":"pm_route","type":"function","z":TAB,"name":"Route status","func":js("route_status.js"),"outputs":9,"timeout":0,"noerr":0,"initialize":"","finalize":"","libs":[],"x":370,"y":220,
     "wires":[["pm_gauge"],["pm_chart"],["pm_rail_txt"],["pm_mode_txt"],["pm_table"],["pm_toast"],["pm_est"],["pm_link"],["pm_wd"]]})
add({"id":"pm_gauge","type":"ui_gauge","z":TAB,"name":"5 V rail","group":G_RAIL,"order":1,"width":3,"height":3,"gtype":"gage","title":"5 V rail","label":"V","format":"{{value}}","min":3.5,"max":5.5,"colors":["#ca3838","#e6a23c","#52c41a"],"seg1":"4.5","seg2":"4.75","diff":False,"className":"","x":620,"y":60,"wires":[]})
add({"id":"pm_chart","type":"ui_chart","z":TAB,"name":"Rail history","group":G_RAIL,"order":3,"width":6,"height":4,"label":"Rail voltage (last 10 min)","chartType":"line","legend":"false","xformat":"HH:mm:ss","interpolate":"linear","nodata":"Waiting for rover/power/status","dot":False,"ymin":"3.5","ymax":"5.5","removeOlder":"10","removeOlderPoints":"","removeOlderUnit":"60","cutout":0,"useOneColor":False,"useUTC":False,"colors":["#1f77b4","#aec7e8","#ff7f0e","#2ca02c","#98df8a","#d62728","#ff9896","#9467bd","#c5b0d5"],"outputs":1,"useDifferentColor":False,"className":"","x":620,"y":100,"wires":[[]]})
add({"id":"pm_rail_txt","type":"ui_text","z":TAB,"group":G_RAIL,"order":2,"width":3,"height":1,"name":"Rail state","label":"Rail:","format":"{{msg.payload}}","layout":"col-center","className":"","x":620,"y":140,"wires":[]})
add({"id":"pm_mode_txt","type":"ui_text","z":TAB,"group":G_MODE,"order":1,"width":6,"height":1,"name":"Current mode","label":"Mode:","format":"{{msg.payload}}","layout":"row-spread","className":"","x":620,"y":180,"wires":[]})
add({"id":"pm_table","type":"ui_template","z":TAB,"group":G_LOAD,"name":"Channel table","order":1,"width":6,"height":3,"format":"<div ng-bind-html=\"msg.payload\"></div>","storeOutMessages":True,"fwdInMessages":False,"resendOnRefresh":True,"templateScope":"local","className":"","x":630,"y":220,"wires":[[]]})
add({"id":"pm_toast","type":"ui_toast","z":TAB,"position":"top right","displayTime":"6","highlight":"","sendall":True,"outputs":0,"ok":"OK","cancel":"","raw":False,"className":"","topic":"","name":"Alerts","x":610,"y":260,"wires":[]})
add({"id":"pm_link","type":"ui_text","z":TAB,"group":G_MODE,"order":2,"width":6,"height":1,"name":"ESP32 link","label":"ESP32 link:","format":"{{msg.payload}}","layout":"row-spread","className":"","x":620,"y":300,"wires":[]})
add({"id":"pm_wd","type":"trigger","z":TAB,"name":"Offline watchdog (10 s)","op1":"","op2":"OFFLINE","op1type":"nul","op2type":"str","duration":"10","extend":True,"overrideDelay":False,"units":"s","reset":"","bytopic":"all","topic":"topic","outputs":1,"x":620,"y":340,"wires":[["pm_link"]]})

# --- battery estimate ---
add({"id":"pm_est","type":"function","z":TAB,"name":"Battery estimate (mAh)","func":js("estimator.js"),"outputs":2,"timeout":0,"noerr":0,"initialize":"","finalize":"","libs":[],"x":390,"y":420,"wires":[["pm_batt_gauge"],["pm_batt_txt"]]})
add({"id":"pm_batt_gauge","type":"ui_gauge","z":TAB,"name":"Estimated charge","group":G_RAIL,"order":4,"width":3,"height":3,"gtype":"gage","title":"Est. charge left","label":"%","format":"{{value}}","min":0,"max":100,"colors":["#ca3838","#e6a23c","#52c41a"],"seg1":"20","seg2":"50","diff":False,"className":"","x":650,"y":400,"wires":[]})
add({"id":"pm_batt_txt","type":"ui_text","z":TAB,"group":G_RAIL,"order":5,"width":6,"height":1,"name":"Runtime estimate","label":"Runtime:","format":"{{msg.payload}}","layout":"row-spread","className":"","x":650,"y":440,"wires":[]})
add({"id":"pm_btn_full","type":"ui_button","z":TAB,"name":"Battery full","group":G_RAIL,"order":6,"width":3,"height":1,"passthru":False,"label":"Power bank recharged","tooltip":"Reset the charge estimate to 100 %","color":"","bgcolor":"","className":"","icon":"battery_full","payload":"{\"reset\":true}","payloadType":"json","topic":"topic","topicType":"msg","x":150,"y":420,"wires":[["pm_est"]]})

# --- power mode buttons ---
def button(id_,name,label,payload,bg,fg,order,icon,y):
    add({"id":id_,"type":"ui_button","z":TAB,"name":name,"group":G_MODE,"order":order,"width":3,"height":1,"passthru":False,"label":label,"tooltip":"","color":fg,"bgcolor":bg,"className":"","icon":icon,"payload":payload,"payloadType":"str","topic":"rover/power/cmd","topicType":"str","x":140,"y":y,"wires":[["pm_cmd_out","pm_sim"]]})
button("pm_btn_active","Active","ACTIVE","active","#28a745","#ffffff",3,"play_arrow",520)
button("pm_btn_sleep","Sleep","SLEEP","sleep","#6c757d","#ffffff",4,"bedtime",560)
button("pm_btn_emerg","Emergency shutdown","EMERGENCY SHUTDOWN","emergency","#dc3545","#ffffff",5,"warning",600)
button("pm_btn_reset","Reset latch","RESET EMERGENCY LATCH","reset","#f0ad4e","#000000",6,"restart_alt",640)
add({"id":"pm_cmd_out","type":"mqtt out","z":TAB,"name":"rover/power/cmd  + req/N","topic":"","qos":"0","retain":"false","respTopic":"","contentType":"","userProps":"","correl":"","expiry":"","broker":BROKER,"x":420,"y":560,"wires":[]})

# --- manual load requests ---
names=["CH1 Intake pump","CH2 Left motor","CH3 Right motor","CH4 Aux pump"]
for i,nm in enumerate(names,1):
    add({"id":"pm_sw%d"%i,"type":"ui_switch","z":TAB,"name":nm,"label":"Request "+nm,"tooltip":"","group":G_LOAD,"order":1+i,"width":6,"height":1,"passthru":False,"decouple":"false","topic":"rover/power/req/%d"%i,"topicType":"str","style":"","onvalue":"1","onvalueType":"str","onicon":"","oncolor":"","offvalue":"0","offvalueType":"str","officon":"","offcolor":"","animate":False,"className":"","x":150,"y":700+40*(i-1),"wires":[["pm_cmd_out","pm_sim"]]})

# --- simulation ---
add({"id":"pm_sim_sw","type":"ui_switch","z":TAB,"name":"Simulate ESP32","label":"Simulate ESP32 (no hardware)","tooltip":"","group":G_SIM,"order":1,"width":6,"height":1,"passthru":True,"decouple":"false","topic":"sim/enable","topicType":"str","style":"","onvalue":"true","onvalueType":"bool","onicon":"","oncolor":"","offvalue":"false","offvalueType":"bool","officon":"","offcolor":"","animate":False,"className":"","x":160,"y":880,"wires":[["pm_sim"]]})
add({"id":"pm_sag_btn","type":"ui_button","z":TAB,"name":"Simulate brownout","group":G_SIM,"order":2,"width":6,"height":1,"passthru":False,"label":"Inject 1.5 s brownout (4.35 V)","tooltip":"","color":"","bgcolor":"#e6a23c","className":"","icon":"flash_off","payload":"1","payloadType":"str","topic":"sim/sag","topicType":"str","x":170,"y":920,"wires":[["pm_sim"]]})
add({"id":"pm_tick","type":"inject","z":TAB,"name":"sim tick 200 ms","props":[{"p":"topic","vt":"str"},{"p":"payload"}],"repeat":"0.2","crontab":"","once":True,"onceDelay":"1","topic":"tick","payload":"1","payloadType":"str","x":160,"y":960,"wires":[["pm_sim"]]})
add({"id":"pm_sim","type":"function","z":TAB,"name":"Simulated ESP32 power manager","func":js("simulator.js"),"outputs":1,"timeout":0,"noerr":0,"initialize":"","finalize":"","libs":[],"x":480,"y":900,"wires":[["pm_route"]]})
add({"id":"pm_dbg","type":"debug","z":TAB,"name":"status (debug)","active":False,"tosidebar":True,"console":False,"tostatus":False,"complete":"payload","targetType":"msg","statusVal":"","statusType":"auto","x":380,"y":120,"wires":[]})
# status tap for debugging
for n in nodes:
    if n["id"]=="pm_mqtt_in": n["wires"][0].append("pm_dbg")
json.dump(nodes,open("power_management_flow.json","w"),indent=2)
print(len(nodes),"nodes")
