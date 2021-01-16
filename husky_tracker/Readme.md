### 以1m的速度进行一下测试, 让机器人前进一米



```python
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped \
'{ header: { frame_id: "base_link" }, pose: { position: { x: 1.0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } }'
```



### 让机器人后退一米，回到原来的位置

```python
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{ header: { frame_id: "map" }, pose: { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } }'

```


