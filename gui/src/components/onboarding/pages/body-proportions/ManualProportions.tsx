import { useForm } from 'react-hook-form';
import { RpcMessage, SkeletonResetAllRequestT } from 'solarxr-protocol';
import { useOnboarding } from '../../../../hooks/onboarding';
import { useWebsocketAPI } from '../../../../hooks/websocket-api';
import { ArrowLink } from '../../../commons/ArrowLink';
import { Button } from '../../../commons/Button';
import { CheckBox } from '../../../commons/Checkbox';
import { PersonFrontIcon } from '../../../commons/PersonFrontIcon';
import { Typography } from '../../../commons/Typography';
import { BodyProportions } from './BodyProportions';
import { useLocalization } from '@fluent/react';

export function ManualProportionsPage() {
  const { l10n } = useLocalization();
  const { applyProgress, skipSetup, state } = useOnboarding();
  const { sendRPCPacket } = useWebsocketAPI();

  applyProgress(0.9);

  const { control, watch } = useForm<{ precise: boolean }>({
    defaultValues: { precise: false },
  });
  const { precise } = watch();

  const resetAll = () => {
    sendRPCPacket(
      RpcMessage.SkeletonResetAllRequest,
      new SkeletonResetAllRequestT()
    );
  };

  return (
    <>
      <div className="flex flex-col gap-5 h-full items-center w-full justify-center">
        <div className="flex flex-col w-full h-full max-w-5xl justify-center">
          <div className="flex gap-8 justify-center">
            <div className="flex flex-col w-full max-w-2xl gap-3 items-center">
              <div className="flex flex-col">
                {!state.alonePage && (
                  <ArrowLink to="/onboarding/reset-tutorial" direction="left">
                    {l10n.getString('onboarding-manual_proportions-back')}
                  </ArrowLink>
                )}
                <Typography variant="main-title">
                  {l10n.getString('onboarding-manual_proportions-title')}
                </Typography>
                <CheckBox
                  control={control}
                  label={l10n.getString(
                    'onboarding-manual_proportions-precision'
                  )}
                  name="precise"
                  variant="toggle"
                ></CheckBox>
              </div>
              <BodyProportions
                precise={precise}
                variant={state.alonePage ? 'alone' : 'onboarding'}
              ></BodyProportions>
            </div>
            <div className="flex-col flex-grow gap-3 rounded-xl fill-background-50 items-center hidden md:flex">
              <PersonFrontIcon width={200}></PersonFrontIcon>
            </div>
          </div>
        </div>
        <div className="w-full py-4 flex flex-row">
          <div className="flex flex-grow gap-3">
            {!state.alonePage && (
              <Button variant="secondary" to="/" onClick={skipSetup}>
                {l10n.getString('onboarding-skip')}
              </Button>
            )}
            <Button variant="secondary" onClick={resetAll}>
              {l10n.getString('reset-reset_all')}
            </Button>
          </div>
          <div className="flex gap-3">
            <Button
              variant="secondary"
              state={{ alonePage: state.alonePage }}
              to="/onboarding/body-proportions/auto"
            >
              {l10n.getString('onboarding-manual_proportions-auto')}
            </Button>
            {!state.alonePage && (
              <Button variant="primary" to="/onboarding/done">
                {l10n.getString('onboarding-continue')}
              </Button>
            )}
          </div>
        </div>
      </div>
    </>
  );
}
