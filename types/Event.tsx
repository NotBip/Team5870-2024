
/**
 * An event object that is passed to the event handlers of the Calendar component.
 * Events have a name, date, an integer that corresponds to a color from a Theme object,
 * and optionally repeat information (repeat ID, start date, end date and weekdays).
 */
export type Event = {
  id: string;
  name: string;
  color: number;

  repeat?: {
    id: string;
    startDate: Date;
    endDate: Date;
    weekdays: number[]; // 0 = Monday, 1 = Tuesday, ..., 6 = Sunday
  };
};

export type EventsOnDate = {
  date: Date;
  events: Event[];
};