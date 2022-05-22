import rclpy
from wd_hga_process.mir import *

from fastapi import FastAPI, Request
import json, socket
import uvicorn

node = ''
app = FastAPI()

@app.get("/")
async def root(self):
    return {"message": "Hello World"}

@app.post('/btn_call')
async def get_info(info: Request): # var_name: var_type
    req_info = await info.json()
    print(req_info['call_id'])
    status = ''
    if req_info['call_id'] == 'q':
            
            status = 'SUCCESS'
    else:
        nCallId = int(req_info['call_id'])
        if nCallId > 0 and nCallId < 4:
            print(nCallId)
            status = 'SUCCESS'
        else:
            print('call_id is out of range.')
            status = 'ERROR: OUT OF RANGE'
    return {
        "status" : status,
        "data" : req_info
    }

def main(args=None):
    global node
    print('Hi from wd_webserver.')
    rclpy.init(args=args)
    node = rclpy.create_node('wd_webserver')
    #uvicorn.run("wd_hga_process.wd_webserver:main", host="0.0.0.0", port=8000, reload=False)
        
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
