import React, { Component, Fragment } from 'react';

import { WithTheme, withTheme } from '@material-ui/core/styles';
import { Avatar, Divider, List, ListItem, ListItemAvatar, ListItemText } from '@material-ui/core';

import DeviceHubIcon from '@material-ui/icons/DeviceHub';
import RefreshIcon from '@material-ui/icons/Refresh';
import ReportIcon from '@material-ui/icons/Report';
import SpeakerNotesIcon from "@material-ui/icons/SpeakerNotes";

import { RestFormProps, FormActions, FormButton, HighlightAvatar } from '../components';
import { mqttStatusHighlight, mqttStatus, mqttPublishHighlight, disconnectReason } from './MqttStatus';
import { MqttStatus } from './types';

type MqttStatusFormProps = RestFormProps<MqttStatus> & WithTheme;

function formatNumber(num: number) {
  return new Intl.NumberFormat().format(num);
}
class MqttStatusForm extends Component<MqttStatusFormProps> {

  renderConnectionStatus() {
    const { data, theme } = this.props
    if (data.connected) {
      return (
        <Fragment>
          <ListItem>
            <ListItemAvatar>
              <Avatar>#</Avatar>
            </ListItemAvatar>
            <ListItemText primary="Client ID" secondary={data.client_id} />
          </ListItem>
          <Divider variant="inset" component="li" />
          <ListItem>
          <ListItemAvatar>
            <HighlightAvatar color={mqttPublishHighlight(data, theme)}>
              <SpeakerNotesIcon />
            </HighlightAvatar>
          </ListItemAvatar>
          <ListItemText
            primary="MQTT Publish Count / Queued / Errors"
            secondary={formatNumber(data.mqtt_count) +' / ' + formatNumber(data.mqtt_queue) + ' / ' + formatNumber(data.mqtt_fails)}
          />
        </ListItem>
        </Fragment>
      );
    }
    return (
      <Fragment>
        <ListItem>
          <ListItemAvatar>
            <Avatar>
              <ReportIcon />
            </Avatar>
          </ListItemAvatar>
          <ListItemText primary="Disconnect Reason" secondary={disconnectReason(data)} />
        </ListItem>
        <Divider variant="inset" component="li" />
      </Fragment>
    );
  }

  createListItems() {
    const { data, theme } = this.props
    return (
      <Fragment>
        <ListItem>
          <ListItemAvatar>
            <HighlightAvatar color={mqttStatusHighlight(data, theme)}>
              <DeviceHubIcon />
            </HighlightAvatar>
          </ListItemAvatar>
          <ListItemText primary="Status" secondary={mqttStatus(data)} />
        </ListItem>
        <Divider variant="inset" component="li" />
        {data.enabled && this.renderConnectionStatus()}
      </Fragment>
    );
  }

  render() {
    return (
      <Fragment>
        <List>
          {this.createListItems()}
        </List>
        <FormActions>
          <FormButton startIcon={<RefreshIcon />} variant="contained" color="secondary" onClick={this.props.loadData}>
            Refresh
          </FormButton>
        </FormActions>
      </Fragment>
    );
  }

}

export default withTheme(MqttStatusForm);
