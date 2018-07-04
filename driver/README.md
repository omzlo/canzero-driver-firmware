
## Notes

Driver can be flashed with the following process:

1) Convert elf firmware to bin

```
arm-none-eabi-objcopy -O binary driver.elf driver.bin
```

2) Upload firmware through SWD connector using st-flash

```
st-flash write driver.bin 0x8000000
``` 
