
import { AppError } from '../types';

export class ErrorHandler {
  static handle(error: Error | unknown, context: string): AppError {
    const message = error instanceof Error ? error.message : 'Unknown error';

    if (message.includes('Failed to fetch') || message.includes('Network Error') || message.includes('fetch')) {
      return {
        message,
        context,
        timestamp: new Date()
      };
    }

    const appError: AppError = {
      message,
      context,
      timestamp: new Date()
    };

    console.error(`[${context}] Error:`, error);
    
    this.notifyUser(appError);
    
    return appError;
  }

  static handleAPIError(error: Error | unknown, endpoint: string): AppError {
    return this.handle(error, `API Call: ${endpoint}`);
  }

  static handleHookError(error: Error | unknown, hookName: string): AppError {
    return this.handle(error, `Hook: ${hookName}`);
  }

  static handleComponentError(error: Error | unknown, componentName: string): AppError {
    return this.handle(error, `Component: ${componentName}`);
  }

  private static notifyUser(error: AppError): void {

    
    if (error.context?.includes('Mission')) {
      this.handleMissionError(error);
    }
  }

  private static handleMissionError(_error: AppError): void {
    // Mission error logic
  }

  static createError(message: string, context?: string): AppError {
    return {
      message,
      context,
      timestamp: new Date()
    };
  }
}

export const handleAPIError = (error: Error | unknown, endpoint: string) => 
  ErrorHandler.handleAPIError(error, endpoint);

export const handleHookError = (error: Error | unknown, hookName: string) => 
  ErrorHandler.handleHookError(error, hookName);

export const handleComponentError = (error: Error | unknown, componentName: string) => 
  ErrorHandler.handleComponentError(error, componentName);