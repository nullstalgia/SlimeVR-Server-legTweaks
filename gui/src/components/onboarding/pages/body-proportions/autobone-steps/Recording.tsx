import { useEffect } from 'react';
import { useAutobone } from '../../../../../hooks/autobone';
import { ProgressBar } from '../../../../commons/ProgressBar';
import { TipBox } from '../../../../commons/TipBox';
import { Typography } from '../../../../commons/Typography';
import { useLocalization } from '@fluent/react';

export function Recording({ nextStep }: { nextStep: () => void }) {
  const { l10n } = useLocalization();
  const { progress, hasCalibration, hasRecording } = useAutobone();

  useEffect(() => {
    if (progress === 1 && hasCalibration) {
      nextStep();
    }
  }, [progress, hasCalibration]);

  return (
    <div className="flex flex-col items-center w-full justify-between">
      <div className="flex gap-1 flex-col justify-center items-center">
        <div className="flex text-status-critical justify-center items-center gap-1">
          <div className="w-2 h-2 rounded-lg bg-status-critical"></div>
          <Typography color="text-status-critical">
            {l10n.getString('onboarding-automatic_proportions-recording-title')}
          </Typography>
        </div>
        <Typography variant="section-title">
          {l10n.getString(
            'onboarding-automatic_proportions-recording-description-p0'
          )}
        </Typography>
        <Typography color="secondary">
          {l10n.getString(
            'onboarding-automatic_proportions-recording-description-p1'
          )}
        </Typography>
      </div>
      <div>
        <Typography color="secondary">
          {l10n.getString('onboarding-automatic_proportions-recording-steps-0')}
        </Typography>
        <Typography color="secondary">
          {l10n.getString('onboarding-automatic_proportions-recording-steps-1')}
        </Typography>
        <Typography color="secondary">
          {l10n.getString('onboarding-automatic_proportions-recording-steps-2')}
        </Typography>
        <Typography color="secondary">
          {l10n.getString('onboarding-automatic_proportions-recording-steps-3')}
        </Typography>
        <Typography color="secondary">
          {l10n.getString('onboarding-automatic_proportions-recording-steps-4')}
        </Typography>
      </div>
      <div className="flex">
        <TipBox>{l10n.getString('tips-do_not_move_heels')}</TipBox>
      </div>
      <div className="flex flex-col gap-2 items-center w-full max-w-[150px]">
        <ProgressBar progress={progress} height={2}></ProgressBar>
        <Typography color="secondary">
          {!hasCalibration && hasRecording
            ? l10n.getString(
                'onboarding-automatic_proportions-recording-processing'
              )
            : l10n.getString(
                'onboarding-automatic_proportions-recording-timer',
                {
                  time: 15,
                }
              )}
        </Typography>
      </div>
    </div>
  );
}
