import { useEffect, useRef, useState } from 'react';
import { useForm } from 'react-hook-form';
import { useLocation } from 'react-router-dom';
import {
  CloseSerialRequestT,
  OpenSerialRequestT,
  RpcMessage,
  SerialDevicesRequestT,
  SerialDevicesResponseT,
  SerialDeviceT,
  SerialTrackerFactoryResetRequestT,
  SerialTrackerGetInfoRequestT,
  SerialTrackerRebootRequestT,
  SerialUpdateResponseT,
} from 'solarxr-protocol';
import { useElemSize, useLayout } from '../../../hooks/layout';
import { useWebsocketAPI } from '../../../hooks/websocket-api';
import { Button } from '../../commons/Button';
import { Dropdown } from '../../commons/Dropdown';
import { Typography } from '../../commons/Typography';
import { useLocalization } from '@fluent/react';

export interface SerialForm {
  port: string;
}

export function Serial() {
  const {
    layoutHeight,
    layoutWidth,
    ref: consoleRef,
  } = useLayout<HTMLDivElement>();
  const { l10n } = useLocalization();

  const { state } = useLocation();

  const toolbarRef = useRef<HTMLDivElement>(null);
  const { height } = useElemSize(toolbarRef);

  const { useRPCPacket, sendRPCPacket } = useWebsocketAPI();
  // const consoleRef = useRef<HTMLPreElement>(null);
  const [consoleContent, setConsole] = useState('');

  const [isSerialOpen, setSerialOpen] = useState(false);
  const [serialDevices, setSerialDevices] = useState<
    Omit<SerialDeviceT, 'pack'>[]
  >([]);

  const { control, watch, handleSubmit, reset } = useForm<SerialForm>({
    defaultValues: { port: 'Auto' },
  });

  const { port } = watch();

  useEffect(() => {
    const subscription = watch(() => handleSubmit(onSubmit)());
    return () => subscription.unsubscribe();
  }, []);

  const onSubmit = (value: SerialForm) => {
    openSerial(value.port);
    setConsole('');
  };

  const openSerial = (port: string) => {
    sendRPCPacket(RpcMessage.CloseSerialRequest, new CloseSerialRequestT());
    const req = new OpenSerialRequestT();
    req.auto = port === 'Auto';
    req.port = port;
    sendRPCPacket(RpcMessage.OpenSerialRequest, req);
  };

  useEffect(() => {
    sendRPCPacket(RpcMessage.SerialDevicesRequest, new SerialDevicesRequestT());
    const typedState: { serialPort: string } = state as any;
    if (typedState?.serialPort) {
      reset({ port: typedState.serialPort });
    }
  }, []);

  useEffect(() => {
    return () => {
      sendRPCPacket(RpcMessage.CloseSerialRequest, new CloseSerialRequestT());
    };
  }, []);

  useRPCPacket(
    RpcMessage.SerialUpdateResponse,
    (data: SerialUpdateResponseT) => {
      if (data.closed) {
        setSerialOpen(false);
      }

      if (!data.closed) {
        setSerialOpen(true);
      }

      if (data.log && consoleRef.current) {
        setConsole((console) => console + data.log);
      }
    }
  );

  useRPCPacket(
    RpcMessage.SerialDevicesResponse,
    (res: SerialDevicesResponseT) => {
      setSerialDevices([
        {
          name: l10n.getString('settings-serial-auto_dropdown_item'),
          port: 'Auto',
        },
        ...(res.devices || []),
      ]);
    }
  );

  useEffect(() => {
    if (consoleRef.current)
      consoleRef.current.scrollTo({
        top: consoleRef.current.scrollHeight,
      });
  }, [consoleContent]);

  useEffect(() => {
    const id = setInterval(() => {
      if (!isSerialOpen) openSerial(port);
      else clearInterval(id);
    }, 1000);

    return () => {
      clearInterval(id);
    };
  }, [isSerialOpen]);

  const reboot = () => {
    sendRPCPacket(
      RpcMessage.SerialTrackerRebootRequest,
      new SerialTrackerRebootRequestT()
    );
  };
  const factoryReset = () => {
    sendRPCPacket(
      RpcMessage.SerialTrackerFactoryResetRequest,
      new SerialTrackerFactoryResetRequestT()
    );
  };
  const getInfos = () => {
    sendRPCPacket(
      RpcMessage.SerialTrackerGetInfoRequest,
      new SerialTrackerGetInfoRequestT()
    );
  };

  return (
    <div className="flex flex-col bg-background-70 h-full p-5 rounded-md">
      <div className="flex flex-col pb-2">
        <Typography variant="main-title">
          {l10n.getString('settings-serial')}
        </Typography>
        <>
          {l10n
            .getString('settings-serial-description')
            .split('\n')
            .map((line, i) => (
              <Typography color="secondary" key={i}>
                {line}
              </Typography>
            ))}
        </>
      </div>
      <div className="bg-background-80 rounded-lg flex flex-col p-2">
        <div
          ref={consoleRef}
          className="overflow-x-auto overflow-y-auto"
          style={{
            height: layoutHeight - height - 30,
            width: layoutWidth - 24,
          }}
        >
          <div className="flex select-text px-3">
            <pre>
              {isSerialOpen
                ? consoleContent
                : l10n.getString('settings-serial-connection_lost')}
            </pre>
          </div>
        </div>
        <div className="" ref={toolbarRef}>
          <div className="border-t-2 pt-2  border-background-60 border-solid m-2 gap-2 flex flex-row">
            <div className="flex flex-grow gap-2">
              <Button variant="quaternary" onClick={reboot}>
                {l10n.getString('settings-serial-reboot')}
              </Button>
              <Button variant="quaternary" onClick={factoryReset}>
                {l10n.getString('settings-serial-factory_reset')}
              </Button>
              <Button variant="quaternary" onClick={getInfos}>
                {l10n.getString('settings-serial-get_infos')}
              </Button>
            </div>

            <div className="flex justify-end">
              <Dropdown
                control={control}
                name="port"
                placeholder={l10n.getString('settings-serial-serial_select')}
                items={serialDevices.map((device) => ({
                  label: device.name?.toString() || 'error',
                  value: device.port?.toString() || 'error',
                }))}
              ></Dropdown>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
